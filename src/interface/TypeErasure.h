#include <memory>

template<typename Interface>
class Base {
protected:
    std::shared_ptr<Interface> impl;

public:
    template <typename Implementation>
    Base(Implementation&& implementation) : impl(std::make_shared<Implementation>(std::forward<Implementation>(implementation))) {}

    template <typename Implementation>
    Base(std::shared_ptr<Implementation> const& implementation) : impl(implementation) {}
    template <typename Implementation>
    Base(std::shared_ptr<Implementation> && implementation) : impl(std::move(implementation)) {}

    Base() = default;

    template<typename Implementation>
    Base& operator=(Implementation&& implementation) { impl = std::shared_ptr<Interface>(std::forward(implementation)); }

    template <typename Implementation>
    Base& operator=(std::shared_ptr<Implementation> implementation) { impl = std::move(implementation); }

    template<typename Implementation>
    Implementation& get_implementation() {
        if (typeid(std::remove_cvref_t<Implementation>) == typeid(*(impl.get())))
            return dynamic_cast<std::remove_cvref_t<Implementation>&>(*(impl.get()));
        else
            throw std::runtime_error("Object cannot be interpreted as the given type");
    }

    template<typename Implementation>
    Implementation& get_implementation() const {
        if (typeid(std::remove_cvref_t<Implementation>) == typeid(*(impl.get())))
            return dynamic_cast<std::remove_cvref_t<Implementation>&>(*(impl.get()));
        else {
            std::cerr << "\n\nrequested type " << typeid(std::remove_cvref_t<Implementation>).name() << "\n" << std::flush;
            std::cerr << "actual type    " << typeid(*(impl.get())).name() << "\n\n" << std::flush;
            throw std::runtime_error("Object cannot be interpreted as the given type");
        }
    }
};