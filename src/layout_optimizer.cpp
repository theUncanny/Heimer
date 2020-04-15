// This file is part of Heimer.
// Copyright (C) 2019 Jussi Lind <jussi.lind@iki.fi>
//
// Heimer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// Heimer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Heimer. If not, see <http://www.gnu.org/licenses/>.

#include "layout_optimizer.hpp"
#include "constants.hpp"
#include "contrib/SimpleLogger/src/simple_logger.hpp"
#include "graph.hpp"
#include "mind_map_data.hpp"
#include "node.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

class Graph;

class LayoutOptimizer::Impl
{
public:
    Impl(MindMapDataPtr mindMapData)
      : m_mindMapData(mindMapData)
    {
    }

    ~Impl() = default;

    void initialize(double aspectRatio, double minEdgeLength)
    {
        juzzlin::L().info() << "Initializing LayoutOptimizer: aspectRatio=" << aspectRatio << ", minEdgeLength=" << minEdgeLength;

        double area = 0;
        for (auto && node : m_mindMapData->graph().getNodes()) {
            area += (node->size().width() + minEdgeLength) * (node->size().height() + minEdgeLength);
        }

        const double height = std::sqrt(area / aspectRatio);
        const double width = area / height;

        const size_t rows = static_cast<size_t>(height / (Constants::Node::MIN_HEIGHT + minEdgeLength)) + 1;
        const size_t cols = static_cast<size_t>(width / (Constants::Node::MIN_WIDTH + minEdgeLength)) + 1;

        // Builds initial layout

        auto nodes = m_mindMapData->graph().getNodes();
        m_layout = std::make_unique<Layout>();
        m_layout->minEdgeLength = minEdgeLength;
        std::map<int, std::shared_ptr<Cell>> nodesToCells; // Used when building connections
        for (size_t j = 0; j < rows; j++) {
            const auto row = std::make_shared<Row>();
            row->rect.x = 0;
            row->rect.y = static_cast<int>(j) * Constants::Node::MIN_HEIGHT;
            for (size_t i = 0; i < cols; i++) {
                const auto cell = std::make_shared<Cell>();
                row->cells.push_back(cell);
                cell->rect.x = row->rect.x + static_cast<int>(i) * Constants::Node::MIN_WIDTH;
                cell->rect.y = row->rect.y;
                cell->rect.h = Constants::Node::MIN_HEIGHT;
                cell->rect.w = Constants::Node::MIN_WIDTH;

                if (!nodes.empty()) {
                    m_layout->all.push_back(cell);
                    cell->node = nodes.back();
                    nodesToCells[cell->node->index()] = cell;
                    nodes.pop_back();
                }
            }
            m_layout->rows.push_back(row);
        }

        // Setup connections

        for (auto && edge : m_mindMapData->graph().getEdges()) {
            const auto cell0 = nodesToCells[edge->sourceNode().index()];
            assert(cell0);
            const auto cell1 = nodesToCells[edge->targetNode().index()];
            assert(cell1);
            cell0->out.push_back(cell1);
            cell1->in.push_back(cell0);
            cell0->all.push_back(cell1);
            cell1->all.push_back(cell0);
        }
    }

    void optimize()
    {
        if (m_layout->all.size() < 2) {
            return;
        }

        double cost = calculateCost();
        const double initialCost = cost;

        juzzlin::L().info() << "Initial cost: " << initialCost;

        std::uniform_real_distribution<double> dist { 0, 1 };

        // TODO: Automatically decide optimal t
        double t = 200;
        while (t > 0.05) {
            double acceptRatio = 0;
            int stuck = 0;
            do {
                double accepts = 0;
                double rejects = 0;

                double sliceCost = cost;
                for (size_t i = 0; i < m_layout->all.size() * 200; i++) {
                    const auto change = planChange();

                    double newCost = cost;
                    newCost -= change.sourceCell->getInAndOutCost();
                    newCost -= change.targetCell->getInAndOutCost();

                    doChange(change);

                    newCost += change.sourceCell->getInAndOutCost();
                    newCost += change.targetCell->getInAndOutCost();

                    const double delta = newCost - cost;
                    if (delta <= 0) {
                        cost = newCost;
                        accepts++;
                    } else {
                        if (dist(m_engine) < std::exp(-delta / t)) {
                            cost = newCost;
                            accepts++;
                        } else {
                            undoChange(change);
                            rejects++;
                        }
                    }
                }

                acceptRatio = accepts / (rejects + 1);
                const double gain = (cost - sliceCost) / sliceCost;
                juzzlin::L().debug() << "Cost: " << cost << " (" << gain * 100 << "%)"
                                     << " acc: " << acceptRatio << " t: " << t;

                if (gain < 0.1) {
                    stuck++;
                } else {
                    stuck = 0;
                }

            } while (stuck < 5);

            t *= 0.5;
        }

        const double gain = (cost - initialCost) / initialCost;
        juzzlin::L().info() << "End cost: " << cost << " (" << gain * 100 << "%)";
    }

    void extract()
    {
        double maxWidth = 0;
        double maxHeight = 0;
        for (size_t j = 0; j < m_layout->rows.size(); j++) {
            const auto row = m_layout->rows.at(j);
            for (size_t i = 0; i < row->cells.size(); i++) {
                const auto cell = row->cells.at(i);
                if (cell) {
                    cell->rect.x += static_cast<int>(i * m_layout->minEdgeLength);
                    maxHeight = std::fmax(maxHeight, cell->rect.y + cell->rect.h);
                    cell->rect.y += static_cast<int>(j * m_layout->minEdgeLength);
                    maxWidth = std::fmax(maxWidth, cell->rect.x + cell->rect.w);
                }
            }
        }

        for (auto && cell : m_layout->all) {
            cell->node->setLocation(
              QPointF(
                Constants::Node::MIN_WIDTH / 2 + cell->rect.x - maxWidth / 2,
                Constants::Node::MIN_HEIGHT / 2 + cell->rect.y - maxHeight / 2));
        }
    }

private:
    double calculateCost() const
    {
        double cost = 0;
        for (auto cell : m_layout->all) {
            cost += cell->getOutCost();
        }
        return cost;
    }

    struct Cell;

    struct Row;

    using CellVector = std::vector<std::shared_ptr<Cell>>;

    struct Change
    {
        enum class Type
        {
            Move,
            Swap
        };

        Type type;

        std::shared_ptr<Cell> sourceCell;

        std::shared_ptr<Cell> targetCell;

        std::shared_ptr<Row> sourceRow;

        std::shared_ptr<Row> targetRow;

        size_t sourceIndex = 0;

        size_t targetIndex = 0;
    };

    void doChange(const Change & change)
    {
        change.sourceRow->cells.at(change.sourceIndex) = change.targetCell;
        change.targetRow->cells.at(change.targetIndex) = change.sourceCell;
        change.sourceCell->pushRect();
        change.sourceCell->rect.x = change.targetRow->rect.x + static_cast<int>(change.targetIndex) * Constants::Node::MIN_WIDTH;
        change.sourceCell->rect.y = change.targetRow->rect.y;
        change.targetCell->pushRect();
        change.targetCell->rect.x = change.sourceRow->rect.x + static_cast<int>(change.sourceIndex) * Constants::Node::MIN_WIDTH;
        change.targetCell->rect.y = change.sourceRow->rect.y;
    }

    void undoChange(const Change & change)
    {
        change.sourceRow->cells.at(change.sourceIndex) = change.sourceCell;
        change.targetRow->cells.at(change.targetIndex) = change.targetCell;
        change.sourceCell->popRect();
        change.targetCell->popRect();
    }

    Change planChange()
    {
        std::uniform_int_distribution<size_t> rowDist { 0, m_layout->rows.size() - 1 };

        Change change;
        change.type = Change::Type::Swap;
        size_t sourceRowIndex = 0;
        size_t targetRowIndex = 0;

        do {
            sourceRowIndex = rowDist(m_engine);
            change.sourceRow = m_layout->rows.at(sourceRowIndex);
            if (change.sourceRow->cells.empty()) {
                continue;
            }
            std::uniform_int_distribution<size_t> sourceCellDist { 0, change.sourceRow->cells.size() - 1 };
            change.sourceIndex = sourceCellDist(m_engine);
            change.sourceCell = change.sourceRow->cells.at(change.sourceIndex);

            targetRowIndex = rowDist(m_engine);
            change.targetRow = m_layout->rows.at(targetRowIndex);
            if (change.targetRow->cells.empty()) {
                continue;
            }
            std::uniform_int_distribution<size_t> targetCellDist { 0, change.targetRow->cells.size() - 1 };
            change.targetIndex = targetCellDist(m_engine);
            change.targetCell = change.targetRow->cells.at(change.targetIndex);

        } while (change.sourceCell == change.targetCell);

        return change;
    }

    MindMapDataPtr m_mindMapData;

    struct Rect
    {
        int x = 0;

        int y = 0;

        int w = 0;

        int h = 0;
    };

    struct Cell
    {
        inline double x() const
        {
            return rect.x + rect.w / 2;
        }

        inline double y() const
        {
            return rect.y + rect.h / 2;
        }

        inline double distance(Cell & other)
        {
            const auto dx = x() - other.x();
            const auto dy = y() - other.y();
            return std::sqrt(dx * dx + dy * dy);
        }

        inline double overlapCost(Cell & c1, Cell & c2)
        {
            const auto x1 = c1.x() - x();
            const auto y1 = c1.y() - y();
            const auto x2 = c2.x() - x();
            const auto y2 = c2.y() - y();

            if (std::fabs(x1 * y2 - x2 * y1) < 0.001) {
                const auto l1 = x1 * x1 + y1 * y1;
                const auto l2 = x2 * x2 + y2 * y2;
                return l1 < l2 ? 2 * std::sqrt(l1) : 2 * std::sqrt(l2);
            }

            return 0;
        }

        inline double getConnectionCost(const CellVector & connections)
        {
            double cost = 0;
            for (auto && cell : connections) {
                cost += distance(*cell);
            }

            return cost;
        }

        inline double getOverlapCost()
        {
            double cost = 0;
            for (size_t i = 0; i < all.size(); i++) {
                const auto outer = all.at(i);
                for (size_t j = i + 1; j < all.size(); j++) {
                    const auto inner = all.at(j);
                    if (outer != inner) {
                        cost += overlapCost(*outer, *inner);
                    }
                }
            }

            return cost;
        }

        inline double getOutCost()
        {
            double overlapCost = getOverlapCost();
            for (auto && dependency : all) {
                overlapCost += dependency->getOverlapCost();
            }
            return overlapCost + getConnectionCost(out);
        }

        inline double getInAndOutCost()
        {
            return getOutCost() + getConnectionCost(in);
        }

        inline void popRect()
        {
            rect = stash;
        }

        inline void pushRect()
        {
            stash = rect;
        }

        CellVector in;

        CellVector out;

        CellVector all;

        NodePtr node;

        Rect rect;

        Rect stash;

        size_t iteration = 0;
    };

    struct Row
    {
        CellVector cells;

        Rect rect;
    };

    using RowVector = std::vector<std::shared_ptr<Row>>;

    struct Layout
    {
        double minEdgeLength = 0;

        CellVector all;

        RowVector rows;
    };

    std::unique_ptr<Layout> m_layout;

    std::mt19937 m_engine;
};

LayoutOptimizer::LayoutOptimizer(MindMapDataPtr mindMapData)
  : m_impl(std::make_unique<Impl>(mindMapData))
{
}

void LayoutOptimizer::initialize(double aspectRatio, double minEdgeLength)
{
    m_impl->initialize(aspectRatio, minEdgeLength);
}

void LayoutOptimizer::optimize()
{
    m_impl->optimize();
}

void LayoutOptimizer::extract()
{
    m_impl->extract();
}

LayoutOptimizer::~LayoutOptimizer() = default;
