#pragma once

#include "query.h"
#include "triangulation/steiner_graph.h"
#include "file-io/triangulation_file_io.h"

#include <fstream>
#include <memory>

class Client {
private:
    class ClientConcept {
    public:
        virtual ~ClientConcept() = default;

        virtual void write_graph_file(std::ostream &output) const = 0;

        virtual void compute_route(int from, int to) = 0;

        virtual void compute_one_to_all(int from) = 0;

        virtual void compute_one_to_all(int from, std::ostream &out) = 0;

        virtual void write_route_file(std::ostream &output) const = 0;

        virtual void write_tree_file(std::ostream &output) const = 0;

        virtual void write_info(std::ostream &output) const = 0;

        virtual void write_graph_stats(std::ostream &output) const = 0;

        virtual void write_beeline(std::ostream &output) const = 0;
    };

    template<typename GraphT, typename RoutingT> requires std::convertible_to<typename RoutingT::graph_type, GraphT>
    class ClientModel : public ClientConcept {
    private:
        GraphT graph;
        RoutingT router;

        std::unique_ptr<Query<GraphT>> query;
        std::unique_ptr<Result<GraphT>> result;
    public:
        ClientModel(GraphT &&graph, RoutingT &&router) : graph{std::move(graph)}, router{std::move(router)} {};

        ClientModel(GraphT &&graph) : graph{std::move(graph)}, router(this->graph) {};

        void write_graph_file(std::ostream &output) const override;

        void compute_route(int from, int to) override;

        void compute_one_to_all(int from) override;

        void compute_one_to_all(int from, std::ostream &output) override;

        void write_route_file(std::ostream &output) const override;

        void write_tree_file(std::ostream &output) const override;

        void write_info(std::ostream &output) const override;

        void write_beeline(std::ostream &output) const override;

        void write_graph_stats(std::ostream &output) const override;
    };

    std::unique_ptr<ClientConcept> pimpl;
public:
    template<typename ...Args>
    void read_graph_file(std::string path, Args... args);


    void write_graph_file(std::ostream output) {
        pimpl->write_graph_file(output);
    }

    void compute_route(int from, int to) { pimpl->compute_route(from, to); };

    void compute_one_to_all(int from) { pimpl->compute_one_to_all(from); };

    void compute_one_to_all(int from, std::ostream &output) { pimpl->compute_one_to_all(from, output); };

    void write_route_file(std::ostream &output) const { pimpl->write_route_file(output); };

    void write_tree_file(std::ostream &output) const { pimpl->write_tree_file(output); };

    void write_beeline_file(std::ostream &output) const { pimpl->write_beeline(output); };

    void write_info(std::ostream &output) const { pimpl->write_info(output); };

    void write_graph_stats(std::ostream &output) const { pimpl->write_graph_stats(output); };
};
