#pragma once
#include <queue>

template <typename T>
class FixedSizeBuffer {
    // a custom queue wrapper implementing a circular buffer 
    private:
        std::queue<T> buffer_;
        size_t max_size_;

    public:
        FixedSizeBuffer(size_t limit): max_size_(limit) {};

        void push(T new_element) {
            if (buffer_.size() >= max_size_) buffer_.pop();

            buffer_.push(new_element);
        }

        size_t size() {return buffer_.size();};
        T& front() {return buffer_.front();};
        T& back()  {return buffer_.back();};
        void pop() {if (!buffer_.empty()) buffer_.pop();};
};