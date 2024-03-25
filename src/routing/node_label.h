#include <utility>
#include "../util/optional.h"

template <typename Impl>
struct node_label {
private:
    Impl _impl;

    static constexpr bool HasDistance = requires(Impl t) {
        t._distance;
    };

    static constexpr bool HasPredecessor = requires(Impl t){
        t._predecessor;
    };

    static constexpr bool HasFaceCrossingPredecessor = requires(Impl t){
        t._face_crossing_predecessor;
    };

    static constexpr bool HasNode = requires(Impl t){
        t._node;
    };

    static constexpr bool HasHeuristic = requires(Impl t) {
        t._heuristic;
    };

public:
    constexpr node_label() : _impl{} {
        if consteval {
            if constexpr (HasDistance) {
                _impl._distance = std::numeric_limits<double>::infinity();
            }
            if constexpr (HasHeuristic) {
                _impl._heuristic = std::numeric_limits<double>::infinity();
            }
            if constexpr (HasPredecessor) {
                _impl._predecessor = optional::none_value<decltype(_impl._predecessor)>;
            }
            if constexpr (HasFaceCrossingPredecessor) {
                _impl._face_crossing_predecessor = optional::none_value<decltype(_impl._face_crossing_predecessor)>;
            }
            if constexpr (HasNode) {
                _impl._node = optional::none_value<decltype(_impl._node)>;
            }
        }
    };

    template<typename... Args>
    constexpr node_label(Args... args) : _impl(std::forward<Args...>(args...)) {}

    constexpr node_label(auto const& node, auto const& parent, auto const& distance ) {
        if constexpr (HasNode)
            _impl._node = node;
        if constexpr (HasPredecessor)
            _impl._predecessor = parent;
        if constexpr (HasDistance)
            _impl._distance = distance;
    }
    constexpr node_label(Impl const& impl) : _impl{impl} {}

    template <typename OtherImpl>
    friend struct node_label;

    template <typename OtherImpl>
    node_label& operator=(node_label<OtherImpl> const& other) {
        using other_type = node_label<OtherImpl>;
        if constexpr (HasNode && other_type::HasNode) {
            _impl._node = other._impl._node;
        }
        if constexpr (HasPredecessor && other_type::HasPredecessor) {
            _impl._predecessor = other._impl._predecessor;
        }
        if constexpr (HasDistance && other_type::HasDistance) {
            _impl._distance = other._impl._distance;
        }
        if constexpr (HasHeuristic) {
            if constexpr (other_type::HasHeuristic) {
                _impl._heuristic = other._impl._heuristic;
            } else if constexpr (other_type::HasDistance) {
                _impl._heuristic = other._impl._distance;
            }
        }

        return *this;
    }

    auto &node() requires HasNode { return _impl._node; }

    auto const &node() const requires HasNode { return _impl._node; }

    auto &predecessor() requires HasPredecessor { return _impl._predecessor; }

    auto const &predecessor() const requires HasPredecessor { return _impl._predecessor; }

    auto &distance() requires HasDistance { return _impl._distance; }

    auto const &distance() const requires HasDistance { return _impl._distance; }

    auto& heuristic() requires HasHeuristic { return _impl._heuristic; }

    auto const& heuristic() const requires HasHeuristic { return _impl._heuristic; }

    // only for reading to prevent confusion
    auto const& value() const {
        if constexpr(HasHeuristic)
            return _impl._heuristic;
        else
            return _impl._distance;
    }

    auto &face_crossing_predecessor() requires HasFaceCrossingPredecessor { return _impl._face_crossing_predecessor; }

    auto const &face_crossing_predecessor() const requires HasFaceCrossingPredecessor { return _impl._face_crossing_predecessor; }
};

template<typename Impl>
constexpr node_label<Impl> optional::none_value<node_label<Impl>> = {};
