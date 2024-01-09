#pragma once

#include "../util/csv.h"
#include "../graph/base_types.h"
#include "../file-io/triangulation_file_io.h"
#include "../file-io/fmi_file_io.h"
#include "../graph/geometry.h"

#include <memory>
#include <fstream>



enum class GraphType {
    STD_GRAPH_DIRECTED,
    STD_GRAPH_UNDIRECTED,
    STEINER_GRAPH_DIRECTED,
    STEINER_GRAPH_UNDIRECTED
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


class Graph : public GraphInterface{
public:

private:
    template<typename GraphT>
    class GraphImplementation : public GraphInterface {
    private:
    public:
        GraphT graph;
        GraphType _type;

        GraphImplementation(GraphT&& graph) : graph{std::move(graph)} {}

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


    std::shared_ptr<GraphInterface> pimpl;

    template <typename GraphT>
    Graph(GraphImplementation<GraphT>&& impl) : pimpl(std::make_shared<GraphImplementation<GraphT>>(std::move(impl))) {};

public:
    Graph() : pimpl(nullptr) {};

    Graph(Graph&&) = default;
    Graph(Graph const&) = default;

    Graph& operator=(Graph&&) = default;

    template <typename GraphT>
    Graph(GraphT&& graph) : Graph( GraphImplementation<GraphT>( std::forward<GraphT>( graph ) ) ) {};

    template<typename ...Args>
    void read_graph_file(std::string path, Args... args);

    template<typename ...Args>
    void read_graph_file(std::string path, double epsilon, Args... args);

    long node_at(coordinate_t& coordinates) const override { return pimpl->node_at(coordinates); };

    void project(Projection projection) override { pimpl->project(projection); };

    void write_graph_file(std::string path) const override { pimpl->write_graph_file(path); }
    void write_graph_file(std::string path, int color, int linewidth) const override { pimpl->write_graph_file(path, color, linewidth); }

    void write_graph_stats(std::ostream &output) const override { pimpl->write_graph_stats(output); };
    void write_graph_stats(table& out) const override { pimpl->write_graph_stats(out); };

    void write_subgraph_file(std::string path, coordinate_t bottom_left, coordinate_t top_right) const override { pimpl->write_subgraph_file(path, bottom_left, top_right); };

    std::size_t node_count() const override { return pimpl->node_count(); }

    std::size_t edge_count() const override { return pimpl->edge_count(); }

    coordinate_t node_coordinates(long node_id) const override { return pimpl->node_coordinates(node_id); };

    GraphType type() const { return pimpl->type(); };

    template<typename T>
    T& get() const {
        if (typeid(GraphImplementation<std::remove_cvref_t<T>>) == typeid(*(pimpl.get())))
            return static_cast<GraphImplementation<std::remove_cvref_t<T>>&>(*(pimpl.get())).graph;
        else
            throw std::runtime_error("Graph cannot be interpreted with this type");
    }
};
