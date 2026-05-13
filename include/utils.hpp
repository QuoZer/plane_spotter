#pragma once
#include <deque>

template <typename T>
class FixedSizeBuffer {
    // a custom deque wrapper implementing a circular buffer 
    private:
    size_t max_size_;
        std::deque<T> buffer_;

    public:
        FixedSizeBuffer(size_t limit): max_size_(limit) {};

        void push(T new_element) {
            if (buffer_.size() >= max_size_) buffer_.pop_front();

            buffer_.push_back(new_element);
        }
        void pop() {if (!buffer_.empty()) buffer_.pop_front();};

        size_t size() {return buffer_.size();};
        T& front() {return buffer_.front();};
        T& back()  {return buffer_.back();};

        T& operator[](size_t ind) {return buffer_[ind];};
};