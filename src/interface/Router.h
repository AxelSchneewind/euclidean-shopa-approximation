#pragma once

#include <iostream>
#include <memory>
#include "Graph.h"
#include "Query.h"


struct RoutingConfiguration {
    bool use_a_star{true};
    bool bidirectional{false};
    bool compact_labels{false};
    bool live_status {true};
};

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
        GraphT const&_graph;

        std::shared_ptr<QueryImplementation<GraphT>> _query_ptr;
        std::shared_ptr<ResultImplementation<GraphT>> _result_ptr;

        RouterT _router;
        RoutingConfiguration _config;

    public:
        ~RouterImplementation() = default;

        RouterImplementation(GraphT const&graph, RouterT&&router) : _graph(graph), _router(std::move(router)) { };

        void compute_route(long from, long to) override;
        void perform_query(Query const& query) override;

        Query query() override { return Query(_query_ptr); };
        Result result() override { return Result(_result_ptr); };
    };

    RoutingConfiguration _config;

public:
    Router() = default;

    Router(Graph const&graph);

    Router(Graph const&graph, RoutingConfiguration const&);

    void compute_route(int from, int to) { impl->compute_route(from, to); };
    void perform_query(Query const& query) { impl->perform_query(query); };

    Query query() { return impl->query(); }
    Result result() { return impl->result(); }
};
