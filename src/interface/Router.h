#pragma once

#include <memory>
#include "Graph.h"
#include "Query.h"


class RouterInterface {
public:
    virtual ~RouterInterface() = default;

    virtual void compute_route(long from, long to) = 0;
    virtual void perform_query(Query const& query) = 0;

    virtual Query query() = 0;

    virtual Result result() = 0;
};


class Router : public Base<RouterInterface> {
private:

    template<typename GraphT, typename RouterT>
    class RouterImplementation : public RouterInterface {
    private:
        std::shared_ptr<GraphT> _graph;

        std::shared_ptr<QueryImplementation<GraphT>> _query_ptr;
        std::shared_ptr<ResultImplementation<GraphT>> _result_ptr;

        RouterT _router;
        RoutingConfiguration _config;

    public:
        ~RouterImplementation() = default;

        RouterImplementation(std::shared_ptr<GraphT> graph, RouterT&&router, RoutingConfiguration const& config) : _graph(graph), _router(std::move(router)), _config(config) { };

        void compute_route(long from, long to) override;
        void perform_query(Query const& query) override;

        Query query() override { return Query(_query_ptr); };
        Result result() override { return Result(_result_ptr); };
    };

    RoutingConfiguration _config;

public:
    Router() = default;

    Router(Graph const&graph, RoutingConfiguration const&);

    void compute_route(int from, int to) { impl->compute_route(from, to); };
    void perform_query(Query const& query) { impl->perform_query(query); };

    Query query() { return impl->query(); }
    Result result() { return impl->result(); }
};
