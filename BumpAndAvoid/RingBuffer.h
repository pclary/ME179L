#ifndef RINGBUFFER_H
#define RINGBUFFER_H


template <typename T, int _capacity>
class RingBuffer 
{
public:
    RingBuffer() 
    {
        headIndex = 0;
        _size = 0;
    }
    T& operator[](int index);
    T read(int index);
    void write(T value, int index);
    void push(T value);
    T pop();
    int size()
    {
        return _size;
    }
    int capacity()
    {
        return _capacity;
    }

private:
    T data[_capacity];
    int headIndex;
    int _size;
    inline int getRealIndex(int index)
    {
        return ((headIndex - index) % _capacity + _capacity) % _capacity;
    }
};


/* read(int index)
 *
 * Returns the object stored at a given index.
 */
template <typename T, int _capacity>
inline T RingBuffer<T, _capacity>::read(int index)
{
    return data[getRealIndex(index)];
}


/* operator[int index]
 *
 * Returns a reference to the object stored at a given index.
 */
template <typename T, int _capacity>
inline T& RingBuffer<T, _capacity>::operator[](int index)
{
    return data[getRealIndex(index)];
}


/* write(T value, int index)
 *
 * Sets the value of the object stored at the given index.
 */
template <typename T, int _capacity>
inline void RingBuffer<T, _capacity>::write(T value, int index)
{
    data[getRealIndex(index)] = value;
}


/* push(T value)
 *
 * Pushes the given value to the head of the buffer.
 * Moves the head forward.
 * Overwrites the oldest value when size == capacity.
 */
template <typename T, int _capacity>
inline void RingBuffer<T, _capacity>::push(T value)
{
    ++headIndex %= _capacity;
    data[headIndex] = value;
    if (_size < _capacity)
        ++_size;
}


/* pop()
 *
 * Pops the top value off of the buffer and returns it.
 * Moves the head backward.
 * Calling pop() when size == 0 results in undefined behavior.
 */
template <typename T, int _capacity>
inline T RingBuffer<T, _capacity>::pop()
{
    int oldHead = headIndex;
    headIndex = (headIndex - 1 + _capacity) % _capacity;
    if (size > 0)
        --_size;
    return data[oldHead]; 
}


#endif // RINGBUFFER_H