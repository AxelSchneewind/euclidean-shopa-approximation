#pragma once

#include "adjacency_list.h"
#include <memory>


template<typename E>
class bidirectional_adjacency_list {
private:
    bidirectional_adjacency_list(std::shared_ptr<const adjacency_list<E> > forward, std::shared_ptr<adjacency_list<E> > backward, std::shared_ptr<std::vector<node_id_t> > forward_ids, std::shared_ptr<std::vector<node_id_t> > backward_ids);

    const std::shared_ptr<const adjacency_list<E> > _forward;
    const std::shared_ptr<const adjacency_list<E> > _backward;

public:
    const std::shared_ptr<const std::vector<node_id_t> > _backward_node_ids;
    const std::shared_ptr<const std::vector<node_id_t> > _forward_node_ids;


    static bidirectional_adjacency_list<E>
    make_bidirectional(const std::shared_ptr<const adjacency_list<E>> &forward) {
        std::shared_ptr<std::vector<node_id_t> > _backward_node_ids(new std::vector<node_id_t>);
        std::shared_ptr<std::vector<node_id_t> > _forward_node_ids(new std::vector<node_id_t>);

        std::shared_ptr<const adjacency_list<E> > _forward(forward);
        std::shared_ptr<adjacency_list<E> > _backward(new adjacency_list<E>(forward->inverse(*_forward_node_ids, *_backward_node_ids)));

        return bidirectional_adjacency_list<E>(_forward, _backward, _forward_node_ids, _backward_node_ids);
    }
    static bidirectional_adjacency_list<E>
    make_bidirectional(adjacency_list<E> &&forward) {
        std::shared_ptr<const adjacency_list<E>> _fwd(new adjacency_list<E>(std::move(forward)));
        return make_bidirectional(_fwd);
    }

    bidirectional_adjacency_list(const adjacency_list<E> &forward) : bidirectional_adjacency_list(std::shared_ptr<const adjacency_list<E> >(&forward)){};
    bidirectional_adjacency_list(bidirectional_adjacency_list<E> &&other)
            : _forward(std::move(other._forward)),
              _backward(std::move(other._backward)),
              _forward_node_ids(std::move(other._forward_node_ids)),
              _backward_node_ids(std::move(other._backward_node_ids)) {};

    bidirectional_adjacency_list inverse() const;

    inline const size_t node_count() const;
    inline const size_t edge_count() const;

    const adjacency_list<E> &
    forward() const { return *_forward; }
    const adjacency_list<E> &
    backward() const { return *_backward; }

    bool operator==(const bidirectional_adjacency_list<E> &other);
};
template<typename E>
bidirectional_adjacency_list<E>::bidirectional_adjacency_list(std::shared_ptr<const adjacency_list<E> > forward, std::shared_ptr<adjacency_list<E> > backward, std::shared_ptr<std::vector<node_id_t> > forward_ids, std::shared_ptr<std::vector<node_id_t> > backward_ids)
    : _forward(forward), _backward(backward), _forward_node_ids(forward_ids), _backward_node_ids(backward_ids) {
}

template<typename E>
bool
bidirectional_adjacency_list<E>::operator==(const bidirectional_adjacency_list<E> &other) {
    return forward() == other.forward();
}

template<typename E>
const size_t
bidirectional_adjacency_list<E>::node_count() const {
    return forward().node_count();
}
template<typename E>
const size_t
bidirectional_adjacency_list<E>::edge_count() const {
    return forward().edge_count();
}

template<typename E>
bidirectional_adjacency_list<E>
bidirectional_adjacency_list<E>::inverse() const { return bidirectional_adjacency_list<E>(_backward, _forward, _backward_node_ids, _forward_node_ids); }