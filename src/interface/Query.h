#pragma once

#include <chrono>
#include <iostream>
#include <ratio>

#include "../graph/base_types.h"

#include "../graph/graph.h"
#include "Graph.h"
#include "Statistics.h"
#include "RoutingConfig.h"


class QueryInterface {
public:
    virtual ~QueryInterface() = default;
    virtual void write(std::ostream& out) const = 0;
    virtual void write(table& out) const = 0;

    virtual long from() const = 0;
    virtual long to() const = 0;

    virtual RoutingConfiguration const& config() const = 0;

    virtual Graph beeline() const = 0;
    virtual distance_t beeline_distance() const = 0;
};

template<RoutableGraph GraphT>
class QueryImplementation : public QueryInterface{
private:
    using node_id_type = typename GraphT::node_id_type;
    long _from;
    long _to;
    node_id_type _from_internal;
    node_id_type _to_internal;

    Graph _beeline;
    distance_t _beeline_distance;

    RoutingConfiguration _configuration;

public:
    QueryImplementation();
    QueryImplementation(GraphT const& graph, long from, long to, RoutingConfiguration const& config);

    long from() const override { return _from; }
    long to() const override { return _to; }

    node_id_type from_internal() const { return _from_internal; }
    node_id_type to_internal() const { return _to_internal; }

    RoutingConfiguration const& config() const override { return _configuration; }

    long max_tree_size() const { return _configuration.tree_size; };

    Graph beeline() const override  { return _beeline; };
    distance_t beeline_distance() const override { return _beeline_distance; }

    void write(std::ostream& out) const override { out <<   "query:                                    " << _from << ',' << _to
                                                       << "\nbeeline distance:                         " << beeline_distance(); }

    void write(table& out) const override {
        out.put(Statistics::FROM, _from);
        out.put(Statistics::FROM_INTERNAL, _from_internal);
        out.put(Statistics::TO, _to);
        out.put(Statistics::TO_INTERNAL, _to_internal);
        out.put(Statistics::BEELINE_DISTANCE, beeline_distance());
    }
};

class Query : public Base<QueryInterface> {
public:
    Query() = default;

    Query(Query const&) = default;
    Query(Query &&) noexcept = default;

    Query& operator=(Query const&) = default;
    Query& operator=(Query &&) noexcept = default;

    template<typename GraphT>
    Query(QueryImplementation<GraphT> const& impl) : Base<QueryInterface>(impl) {}

    template<typename GraphT>
    Query(QueryImplementation<GraphT> && impl) : Base<QueryInterface>(std::move(impl)) {}

    template<typename GraphT>
    Query(std::shared_ptr<QueryImplementation<GraphT>> impl) : Base<QueryInterface>(std::move(impl)) {}

    template<typename GraphT>
    Query(GraphT& graph, long from, long to, RoutingConfiguration const& config) : Base<QueryInterface>(QueryImplementation<GraphT>{graph, from, to, config}) {}

    Graph beeline() const { return impl->beeline(); };
    distance_t beeline_distance() const { return impl->beeline_distance(); };

    void write(std::ostream& out) const { impl->write(out); out << std::endl; };
    void write(table& out) const { impl->write(out); };

    RoutingConfiguration const& config() const { return impl->config(); }

    long from() const { return impl->from(); }
    long to() const { return impl->to(); }
};




class ResultInterface {
public:
    virtual ~ResultInterface() = default;

    virtual Query query() const = 0;

    virtual bool route_found() const = 0;

    virtual distance_t distance() const = 0;

    virtual Graph & tree_forward() = 0;
    virtual Graph & tree_backward() = 0;
    virtual Graph & path() = 0;

    virtual Graph const& tree_forward() const = 0;
    virtual Graph const& tree_backward() const = 0;
    virtual Graph const& path() const = 0;

    virtual size_t nodes_visited() const = 0;
    virtual size_t edges_visited() const = 0;
    virtual size_t pull_count() const = 0;
    virtual size_t push_count() const = 0;
    virtual size_t queue_max_size() const = 0;

    virtual std::chrono::duration<double, std::milli> duration() const = 0;

    virtual void write(table& out) const = 0;
};

template<RoutableGraph GraphT>
struct ResultImplementation : public ResultInterface {
private:
    QueryImplementation<GraphT> _query;

    bool _route_found = false;

    distance_t _distance = infinity<distance_t>;
    std::chrono::duration<double, std::milli> _duration;

    Graph _path;
    Graph _tree_forward;
    Graph _tree_backward;

    std::size_t _nodes_visited{0};
    std::size_t _edges_visited{0};
    std::size_t _pull_count{0};
    std::size_t _push_count{0};
    std::size_t _queue_max_size{0};

    std::size_t _base_node_count{0};
    std::size_t _boundary_node_count{0};
    std::size_t _steiner_point_count{0};

    std::size_t _base_node_neighbor_count{0};
    std::size_t _boundary_node_neighbor_count{0};
    std::size_t _steiner_point_neighbor_count{0};
    std::size_t _steiner_point_angle_check_count{0};

public:
    ResultImplementation() = default;

    ResultImplementation(ResultImplementation const&) = default;
    ResultImplementation(ResultImplementation &&)  noexcept = default;

    ResultImplementation& operator=(ResultImplementation const&) = default;
    ResultImplementation& operator=(ResultImplementation &&)  noexcept = default;

    template<typename RouterT>
    ResultImplementation(GraphT const& graph, QueryImplementation<GraphT> query, RouterT const& router, std::chrono::duration<double, std::milli> duration);

    Query query() const override { return Query( QueryImplementation<GraphT>( _query ) ) ; };

    bool route_found() const override { return _route_found; };

    distance_t distance() const override { return _distance; };

    Graph& tree_forward() override  { return _tree_forward; };
    Graph& tree_backward() override  { return _tree_backward; };
    Graph& path() override { return _path; };
    Graph const& tree_forward() const override { return _tree_forward; };
    Graph const& tree_backward() const override { return _tree_backward; };
    Graph const& path() const override { return _path; };

    std::size_t nodes_visited() const override { return _nodes_visited; };
    std::size_t edges_visited() const override { return _edges_visited; };
    std::size_t pull_count() const override { return _pull_count; };
    std::size_t push_count() const override { return _push_count; };
    std::size_t queue_max_size() const override { return _queue_max_size; };

    std::chrono::duration<double, std::milli> duration() const override { return _duration; };

    void write(table& out) const override;
};


class Result : public Base<ResultInterface>{
public:
    Result() = default;

    Result(Result&&) noexcept =  default;
    Result& operator=(Result&&) noexcept =  default;

    Result(Result const&) noexcept =  default;
    Result& operator=(Result const&) noexcept =  default;

    template <typename GraphT>
    Result(ResultImplementation<GraphT> const&impl) : Base<ResultInterface>(impl){}

    template <typename GraphT>
    Result(ResultImplementation<GraphT> &&impl) : Base<ResultInterface>(std::move(impl)){}

    template <typename GraphT>
    Result(std::shared_ptr<ResultImplementation<GraphT>> impl) : Base<ResultInterface>(std::move(impl)){}

    template <typename GraphT, typename RouterT>
    Result(GraphT const& graph, QueryImplementation<GraphT> query, RouterT const& router, std::chrono::duration<double, std::milli> duration);

    Query query() const { return impl->query(); }

    bool route_found() const { return impl->route_found(); }

    distance_t distance() const { return impl->distance(); }

    Graph & tree_forward() { return impl->tree_forward(); }
    Graph & tree_backward() { return impl->tree_backward(); }
    Graph & path() { return impl->path(); }
    Graph const& tree_forward() const { return impl->tree_forward(); }
    Graph const& tree_backward() const { return impl->tree_backward(); }
    Graph const& path() const { return impl->path(); }

    std::size_t nodes_visited() const { return impl->nodes_visited(); }
    std::size_t edges_visited() const { return impl->edges_visited(); }
    std::size_t pull_count() const { return impl->pull_count(); }
    std::size_t push_count() const { return impl->push_count(); }

    std::chrono::duration<double, std::milli> duration() const { return impl->duration(); }

    void write(table& out) const { impl->write(out); }
};
