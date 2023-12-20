#pragma once

#include <chrono>
#include <iostream>

#include "../graph/base_types.h"

#include "../graph/graph.h"
#include "Graph.h"


class QueryInterface {
public:
    virtual ~QueryInterface() = default;
    virtual void write(std::ostream& out) const = 0;

    virtual long from() const = 0;
    virtual long to() const = 0;
};

template<RoutableGraph GraphT>
class QueryImplementation : public QueryInterface{
private:
    using node_id_type = typename GraphT::node_id_type;
    long _from;
    long _to;
    node_id_type _from_internal;
    node_id_type _to_internal;

public:
    QueryImplementation();;
    QueryImplementation(GraphT const& graph, long from, long to);

    long from() const { return _from; }
    long to() const { return _to; }

    node_id_type from_internal() const { return _from_internal; }
    node_id_type to_internal() const { return _to_internal; }

    void write(std::ostream& out) const { out << _from << ',' << _to; }
};

class Query {
private:
    std::shared_ptr<QueryInterface> pimpl;
public:
    Query() : pimpl(nullptr) {}

    Query(Query const&) = default;
    Query(Query &&) noexcept = default;

    Query& operator=(Query const&) = default;
    Query& operator=(Query &&) noexcept = default;

    template<typename GraphT>
    Query(QueryImplementation<GraphT> const& impl) : pimpl(std::make_shared<QueryImplementation<GraphT>>(impl)) {}

    template<typename GraphT>
    Query(QueryImplementation<GraphT> && impl) : pimpl(std::make_shared<QueryImplementation<GraphT>>(impl)) {}

    template<typename GraphT>
    Query(GraphT& graph, long from, long to) : pimpl{graph, from, to} {}

    Query(Query&) = default;
    Query& operator=(Query&) = default;

    void write(std::ostream& out) const { out << pimpl->from() << ',' << pimpl->to(); };

    long from() const { return pimpl->from(); }
    long to() const { return pimpl->to(); }
};




class ResultInterface {
public:
    virtual ~ResultInterface() = default;

    virtual Query query() const = 0;

    virtual bool route_found() const = 0;

    virtual distance_t distance() const = 0;
    virtual distance_t beeline_distance() const = 0;

    virtual Graph const& tree_forward() const = 0;
    virtual Graph const& tree_backward() const = 0;
    virtual Graph const& path() const = 0;

    virtual size_t nodes_visited() const = 0;

    virtual std::chrono::duration<double, std::milli> duration() const = 0;
};

template<RoutableGraph GraphT>
struct ResultImplementation : public ResultInterface {
private:
    QueryImplementation<GraphT> _query;

    bool _route_found = false;

    distance_t _distance = infinity<distance_t>;
    distance_t _beeline_distance = infinity<distance_t>;

    Graph _tree_forward;
    Graph _tree_backward;
    Graph _path;

    std::size_t _nodes_visited = 0;

    std::chrono::duration<double, std::milli> _duration;

public:
    ResultImplementation() = default;

    ResultImplementation(ResultImplementation const&) = default;
    ResultImplementation(ResultImplementation &&) = default;

    ResultImplementation& operator=(ResultImplementation const&) = default;
    ResultImplementation& operator=(ResultImplementation &&) = default;

    template<typename RouterT>
    ResultImplementation(GraphT const& graph, QueryImplementation<GraphT> query, RouterT const& router, std::chrono::duration<double, std::milli> duration);;

    Query query() const { return Query( QueryImplementation<GraphT>( _query ) ) ; };

    bool route_found() const { return _route_found; };

    distance_t distance() const { return _distance; };
    distance_t beeline_distance() const { return _beeline_distance; };

    Graph const & tree_forward() const { return _tree_forward; };
    Graph const & tree_backward() const { return _tree_backward; };
    Graph const & path() const { return _path; };

    std::size_t nodes_visited() const { return _nodes_visited; };

    std::chrono::duration<double, std::milli> duration() const { return _duration; };
};

class Result {
private:
    std::shared_ptr<ResultInterface> pimpl;
public:
    Result() = default;

    Result(Result&&) noexcept =  default;
    Result& operator=(Result&&) noexcept =  default;

    Result(Result const&) noexcept =  default;
    Result& operator=(Result const&) noexcept =  default;

    template <typename GraphT>
    Result(ResultImplementation<GraphT> const&impl) : pimpl(std::make_shared<ResultImplementation<GraphT>>(impl)){};

    template <typename GraphT>
    Result(ResultImplementation<GraphT> &&impl) : pimpl(std::make_shared<ResultImplementation<GraphT>>(std::move(impl))){};

    template <typename GraphT>
    Result(std::shared_ptr<ResultImplementation<GraphT>> impl) : pimpl(std::move(impl)){};

    template <typename GraphT, typename RouterT>
    Result(GraphT const& graph, QueryImplementation<GraphT> query, RouterT const& router, std::chrono::duration<double, std::milli> duration);

    Query query() const { return pimpl->query(); };

    bool route_found() const { return pimpl->route_found(); };

    distance_t distance() const { return pimpl->distance(); };
    distance_t beeline_distance() const { return pimpl->beeline_distance(); };

    Graph const& tree_forward() const { return pimpl->tree_forward(); };
    Graph const& tree_backward() const { return pimpl->tree_backward(); };
    Graph const& path() const { return pimpl->path(); };

    std::size_t nodes_visited() const { return pimpl->nodes_visited(); };

    std::chrono::duration<double, std::milli> duration() const { return pimpl->duration(); };
};
