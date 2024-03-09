#pragma once

#include "../util/csv.h"
#include "../graph/base_types.h"
#include "../file-io/triangulation_file_io.h"
#include "../file-io/fmi_file_io.h"
#include "../graph/geometry.h"
#include "TypeErasure.h"

#include <memory>
#include <fstream>


enum class GraphType {
    NONE,
    STD_GRAPH,
    STEINER_GRAPH,
};

class GraphInterface {
public:
    virtual ~GraphInterface() = default;

    virtual void project(Projection projection) = 0;

    virtual void write_graph_file(std::string path) const = 0;
    virtual void write_graph_file(std::string path, int color, int linewidth) const = 0;

    virtual void write_subgraph_file(std::string path, coordinate_t bottom_left, coordinate_t top_right) const = 0;

    virtual void write_graph_stats(std::ostream &output) const = 0;
    virtual void write_graph_stats(table &out) const = 0;

    virtual std::size_t node_count() const = 0;
    virtual std::size_t edge_count() const = 0;

    virtual coordinate_t node_coordinates(long node_id) const = 0;
    virtual long node_at(coordinate_t& coordinates) const = 0;

    virtual GraphType type() const = 0;
};


class Graph : public Base<GraphInterface>, public GraphInterface{
public:

private:
    template<typename GraphT>
    class GraphImplementation : public GraphInterface {
    private:
    public:
        std::shared_ptr<GraphT> graph;
        GraphType _type {GraphType::NONE};

        GraphImplementation(GraphT&& graph) : graph{std::make_shared<GraphT>(std::move(graph))} {}

        GraphImplementation(GraphImplementation&&) noexcept = default;
        GraphImplementation& operator=(GraphImplementation&&) noexcept = default;

        ~GraphImplementation() = default;

        void project(Projection projection) override;

        void write_graph_file(std::string path) const override;
        void write_graph_file(std::string path, int color, int linewidth) const override;

        void write_subgraph_file(std::string path, coordinate_t bottom_left, coordinate_t top_right) const override;;

        void write_graph_stats(std::ostream &output) const override;
        void write_graph_stats(table &out) const override;

        std::size_t node_count() const override;

        std::size_t edge_count() const override;

        coordinate_t node_coordinates(long node_id) const override;

        long node_at(coordinate_t& position) const override;

        GraphType type() const override;
    };


    template <typename GraphT>
    Graph(GraphImplementation<GraphT>&& impl) : Base<GraphInterface>(std::move(impl)) {}

public:
    Graph() = default;

    Graph(Graph&&) = default;
    Graph(Graph const&) = default;

    Graph& operator=(Graph&&) = default;

    template <typename GraphT>
    Graph(GraphT&& graph) : Graph(GraphImplementation<GraphT>( std::forward<GraphT>( graph ) ) ) {}

    template<typename ...Args>
    void read_graph_file(std::string path, Args... args);

    template<typename ...Args>
    void read_graph_file(std::string path, double epsilon, Args... args);

    long node_at(coordinate_t& coordinates) const override { return impl->node_at(coordinates); }

    void project(Projection projection) override { impl->project(projection); }

    void write_graph_file(std::string path) const override { impl->write_graph_file(path); }
    void write_graph_file(std::string path, int color, int linewidth) const override { impl->write_graph_file(path, color, linewidth); }

    void write_graph_stats(std::ostream &output) const override { impl->write_graph_stats(output); }
    void write_graph_stats(table& out) const override { impl->write_graph_stats(out); }

    void write_subgraph_file(std::string path, coordinate_t bottom_left, coordinate_t top_right) const override { impl->write_subgraph_file(path, bottom_left, top_right); }

    std::size_t node_count() const override { return impl->node_count(); }

    std::size_t edge_count() const override { return impl->edge_count(); }

    coordinate_t node_coordinates(long node_id) const override { return impl->node_coordinates(node_id); }

    GraphType type() const override { return impl->type(); }

    template<typename G>
    std::shared_ptr<G> get_implementation() {
        return Base<GraphInterface>::get_implementation<GraphImplementation<G>>().graph;
    }

    template<typename G>
    std::shared_ptr<G> get_implementation() const {
        return Base<GraphInterface>::get_implementation<GraphImplementation<G>>().graph;
    }
};
