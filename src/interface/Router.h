#pragma once

#include <iostream>
#include <memory>
#include "Graph.h"
#include "Query.h"

class Router {

private:
    class RouterInterface {
    public:
        virtual ~RouterInterface() = default;

        virtual void compute_route(long from, long to) = 0;

        virtual Query query() = 0;
        virtual Result result() = 0;
    };


    template <typename GraphT, typename RouterT>
    class RouterImplementation : public RouterInterface {
    private:
        Graph const& _graph;
        QueryImplementation<GraphT> _query;
        ResultImplementation<GraphT> _result;

        std::shared_ptr<QueryImplementation<GraphT>> _query_ptr;
        std::shared_ptr<ResultImplementation<GraphT>> _result_ptr;

        RouterT _router;
    public:
        ~RouterImplementation() = default;

        RouterImplementation(GraphT && graph, RouterT && router) : _graph{std::move(graph)}, _router{std::move(router)} {};
        RouterImplementation(Graph const& graph, RouterT && router) : _graph{graph}, _router{std::move(router)} {};

        void compute_route(long from, long to) override;

        Query query() override { return Query(_query); };
        Result result() override { return Result(_result_ptr); };
    };

    std::unique_ptr<RouterInterface> pimpl;

public:
    Router() = default;

    template<typename RouterT>
    Router(Graph const& graph, RouterT&& router) : pimpl{graph, std::forward(router)} {};

    Router(Graph const& graph);

    void compute_route(int from, int to) { pimpl->compute_route(from, to); };

    Query query() { return pimpl->query(); }
    Result result() { return pimpl->result(); }
};
