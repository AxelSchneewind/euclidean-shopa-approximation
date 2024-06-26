#pragma once

#include "base_types.h"
#include "../util/counting_iterator.h"
#include "../util/contract.h"

#include <array>
#include <span>
#include <vector>



template<typename NodeId, typename E = void>
struct adjacency_list_edge {
    NodeId source;
    NodeId destination;
    E info;

    template <typename... Args>
    adjacency_list_edge(NodeId source, NodeId dest, Args&&... args) : source{source}, destination{dest}, info(std::forward<Args...>(args...)){}

    adjacency_list_edge(NodeId source, NodeId dest) : source{source}, destination{dest}, info{} {}

    adjacency_list_edge() = default;

    bool operator==(const adjacency_list_edge &) const = default;

    bool operator!=(const adjacency_list_edge &) const = default;
};

template<typename NodeId, typename E = void>
struct internal_adjacency_list_edge {
    // TODO rename members and add getters
    NodeId destination;
    E info;

    template <typename OtherN, typename OtherE>
    internal_adjacency_list_edge(adjacency_list_edge<OtherN, OtherE> const& other) : destination{other.destination}, info{other.info} {}

    template <typename OtherN>
    internal_adjacency_list_edge(OtherN const& other) : destination{other} {}
};


template<typename NodeId>
struct internal_adjacency_list_edge<NodeId, void> {
    NodeId destination;

    internal_adjacency_list_edge() = default;

    template<typename... Args>
    internal_adjacency_list_edge(NodeId destination, Args&&...  /*ignore*/) : destination(destination) {}

    template <typename OtherN, typename OtherE>
    internal_adjacency_list_edge(adjacency_list_edge<OtherN, OtherE> const& other) : destination{other.destination} {}

    template <typename OtherN>
    internal_adjacency_list_edge(OtherN const& other) : destination{other} {}
};

template<typename NodeId>
struct adjacency_list_edge<NodeId, void> {
    NodeId source;
    NodeId destination;

    adjacency_list_edge() = default;

    template<typename... Args>
    adjacency_list_edge(NodeId source, NodeId destination, Args&&...  /*ignore*/) : source(source),
                                                                             destination(destination) {}

    adjacency_list_edge(NodeId source, NodeId destination) : source(source), destination(destination) {}


    bool operator==(const adjacency_list_edge &) const = default;

    bool operator!=(const adjacency_list_edge &) const = default;

    operator internal_adjacency_list_edge<NodeId>() const { return {destination}; }
};

/**
 * stores a directed graph. Provides O(1) access to outgoing edges for any node
 * @tparam NodeId the type by which nodes are identified
 * @tparam E information stored on each edge
 */
template<typename NodeId, typename E = void>
class unidirectional_adjacency_list {
public:
    using edge_index_type = edge_id_t;
    using node_id_type = NodeId;
    using edge_info_type = E;

    class adjacency_list_builder {
    private:
        std::size_t _node_count{0};
        std::size_t _edge_count{0};

        std::vector<adjacency_list_edge<NodeId, E>> _edges;
        std::vector<edge_index_type> _offsets;

        bool _edges_sorted{false};
        bool _offsets_valid{false};

        [[gnu::cold]]
        void make_offsets();

        [[gnu::cold]]
        void sort_edges();

        [[gnu::cold]]
        void remove_duplicates();

    public:
        using edge_type = adjacency_list_edge<NodeId, E>;

        adjacency_list_builder() {};

        adjacency_list_builder(adjacency_list_builder &&other) = default;

        adjacency_list_builder(const adjacency_list_builder &other) = default;

        adjacency_list_builder(std::size_t node_count) : _node_count(node_count) { _edges.reserve(4 * _node_count); _offsets.reserve(node_count); };

        ~adjacency_list_builder() = default;

        adjacency_list_builder &operator=(adjacency_list_builder &&) = default;

        adjacency_list_builder &operator=(const adjacency_list_builder &) = default;

        edge_type &edge(std::size_t index) {
            _offsets_valid = false;
            _edges_sorted = false;
            return _edges[index];
        }

        /**
         * removes nodes that have no incident edges
         */
        void remove_unconnected_nodes();

        void finalize() { sort_edges(); remove_duplicates(); _edges.shrink_to_fit(); };

        template<std::predicate<node_id_type> NodePredicate>
        void filter_nodes(NodePredicate &&node_predicate);

        template<std::predicate<edge_info_type> EdgePredicate>
        void filter_edges(EdgePredicate &&edge_predicate);

        /**
         * permutes the order of nodes and adjusts the edge list accordingly
         * @param new_node_ids
         */
        void permute_nodes(std::span<node_id_type> new_node_ids);

        std::span<edge_type> edges() {
            finalize();
            return {_edges.begin(), _edges.end()};
        }

        /**
         * makes edges from the given faces, such that each edge (v,w) has v < w
         * @param faces
         */
        void add_edges_from_triangulation(std::vector<std::array<node_id_type, 3>> const &faces);

        /**
         * makes edges from the given faces, such that each edge (v,w) has v < w
         * @param faces
         */
        void add_edges_from_triangulation(std::vector<std::array<node_id_type, 3>> &&faces);

        void add_edges(std::vector<edge_type> const &edges);

        void add_edges(std::vector<edge_type> &&edges);

        void add_node(NodeId node);

        void add_edge(adjacency_list_edge<NodeId, E> const &edge) { _edges.emplace_back(edge); };

        template<typename... Args>
        void add_edge(NodeId source, NodeId destination, Args&&... info_args);

        void add_edge(NodeId source, NodeId destination);

        /**
         * for each edge (v,w), inserts (w,v)
         */
        void insert_backward_edges();

        [[gnu::cold]]
        unidirectional_adjacency_list<NodeId, E> get();
    };

    static constexpr std::size_t SIZE_PER_NODE = sizeof(edge_index_type);
    static constexpr std::size_t SIZE_PER_EDGE = sizeof(NodeId) + sizeof(internal_adjacency_list_edge<NodeId, E>);

private:
    std::size_t _M_node_count {0};
    std::size_t _M_edge_count {0};

    // per node
    std::vector<edge_index_type> _M_offsets;

    // per edge
    std::vector<NodeId> _M_sources;
    std::vector<internal_adjacency_list_edge<NodeId, E> > _M_edges;

    inline edge_index_type offset(NodeId node) const;

    inline edge_index_type offset_next(NodeId node) const;


public:
    ~unidirectional_adjacency_list();

    unidirectional_adjacency_list() = default;

    unidirectional_adjacency_list(std::size_t node_count,
                                  std::vector<adjacency_list_edge<NodeId, E> > &&edges);

    unidirectional_adjacency_list(std::vector<edge_id_t> &&offsets,
                                  std::vector<NodeId> &&sources,
                                  std::vector<internal_adjacency_list_edge<NodeId, E>> &&edges);

    // move constructor
    unidirectional_adjacency_list(unidirectional_adjacency_list &&other) noexcept;

    unidirectional_adjacency_list<NodeId, E> &operator=(unidirectional_adjacency_list<NodeId, E> &&other) = default;

    // copy constructor
    unidirectional_adjacency_list(const unidirectional_adjacency_list &other) = delete;

    unidirectional_adjacency_list<NodeId, E> &
    operator=(const unidirectional_adjacency_list<NodeId, E> &other) = delete;

    unidirectional_adjacency_list<NodeId, E> inverse() const;

    inline bool contains_node(node_id_type node_id) const;

    inline bool contains_edge(edge_index_type edge_index) const;


    /**
     * get the number of nodes of this adjacency list
     * @return
     */
    inline std::size_t node_count() const;

    /**
     * get the number of edges stored in this adjacency list
     * @return
     */
    inline std::size_t edge_count() const;

    /**
     * enumerates all edge ids
     * @return
     */
    counter<edge_index_type> edge_ids() const {
        return {(edge_index_type)edge_count()};
    }

    /**
     * enumerates all node ids
     * @return
     */
    counter<edge_index_type> node_ids() const {
        return {node_count()};
    }

    /**
     * get the id of the source node for the edge with given index
     * @param edge
     * @return
     */
    inline NodeId source(edge_index_type edge) const;

    /**
     * get the id of the destination for the edge with given index
     * @param edge
     * @return
     */
    inline NodeId destination(edge_index_type edge) const;

    /**
     * get the edge information at the given index
     * @param edge
     * @return
     */
    inline E edge(edge_index_type edge) const;

    /**
     * get the index of (source, destination)
     * @param source
     * @param dest
     * @return the index, or -1 otherwise
     */
    [[gnu::hot]]
    inline edge_index_type edge_id(NodeId source, NodeId dest) const;

    /**
     * get the index of any edge (source, w) in graph
     * @param source
     * @return
     */
    [[gnu::hot]]
    inline edge_index_type edge_id(NodeId source) const;

    /**
     * check if (source, destination) in graph
     * @param source
     * @param dest
     * @return
     */
    [[gnu::hot]]
    inline bool has_edge(NodeId source, NodeId dest) const;

    /**
     * gets the distance info pairs for outgoing edges from the given node
     * @param node the source node
     * @return a span over the destination/cost pairs
     */
    inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    outgoing_edges(NodeId node) const;

    bool operator==(const unidirectional_adjacency_list<NodeId, E> &other);
};


template<typename NodeId, typename E>
class std::hash<adjacency_list_edge<NodeId, E>>;