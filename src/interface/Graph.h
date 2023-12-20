#pragma once

#include "../graph/base_types.h"
#include "../file-io/triangulation_file_io.h"
#include "../file-io/fmi_file_io.h"

#include <memory>
#include <fstream>


enum class Projection {
    NONE,
    WGS84_TO_GB,
    GB_TO_WGS84
};

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

    virtual void write_graph_file(std::ostream &output) const = 0;

    virtual void write_graph_file_gl(std::ostream &output) const = 0;

    virtual void write_subgraph_file_gl(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const = 0;

    virtual void write_subgraph_file(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const = 0;

    virtual void write_graph_stats(std::ostream &output) const = 0;

    virtual int node_count() const = 0;

    virtual int edge_count() const = 0;

    virtual coordinate_t node_coordinates(int node_id) const = 0;

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

        void write_graph_file(std::ostream &output) const override;

        void write_graph_file_gl(std::ostream &output) const override;

        void write_subgraph_file_gl(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const override {/**/};

        void write_subgraph_file(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const override {/**/};

        void write_graph_stats(std::ostream &output) const override;

        int node_count() const override;

        int edge_count() const override;

        coordinate_t node_coordinates(int node_id) const override;

        GraphType type() const override;
    };


    std::unique_ptr<GraphInterface> pimpl;

    template <typename GraphT>
    Graph(GraphImplementation<GraphT>&& impl) : pimpl(std::make_unique<GraphImplementation<GraphT>>(std::move(impl))) {};

public:
    Graph() : pimpl(nullptr) {};

    Graph(Graph&&) = default;

    Graph& operator=(Graph&&) = default;

    template <typename GraphT>
    Graph(GraphT&& graph) : Graph( GraphImplementation<GraphT>( std::move( graph ) ) ) {};

    template<typename ...Args>
    static Graph read_graph_file(std::string path, Args... args);

    template<typename ...Args>
    static Graph read_graph_file(std::string path, double epsilon, Args... args);

    void project(Projection projection) override { pimpl->project(projection); };

    void write_graph_file(std::ostream &output) const override { pimpl->write_graph_file(output); }
    void write_graph_file_gl(std::ostream &output) const override { pimpl->write_graph_file_gl(output); }

    void write_graph_stats(std::ostream &output) const override { pimpl->write_graph_stats(output); };

    void write_subgraph_file(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const { pimpl->write_subgraph_file(output, bottom_left, top_right); };

    void write_subgraph_file_gl(std::ostream &output, coordinate_t bottom_left, coordinate_t top_right) const { pimpl->write_subgraph_file_gl(output, bottom_left, top_right); };

    int node_count() const override { return pimpl->node_count(); }

    int edge_count() const override { return pimpl->edge_count(); }

    coordinate_t node_coordinates(int node_id) const override { return pimpl->node_coordinates(node_id); };

    GraphType type() const { return pimpl->type(); };

    template<typename T>
    T const& get() const {
        if (typeid(GraphImplementation<T>) == typeid(*pimpl))
            return static_cast<T const&>(static_cast<GraphImplementation<T> const&>(*pimpl).graph);
        else
            throw std::runtime_error("Graph cannot be interpreted with this type");
    }
};
