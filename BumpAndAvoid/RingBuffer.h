#ifndef RINGBUFFER_H
#define RINGBUFFER_H


template <typename T, int capacity>
class RingBuffer 
{
public:
    RingBuffer() 
    {
        headIndex = 0;
        size = 0;
    }
    T& operator[](int index);
    T read(int index);
    void write(T value, int index);
    void push(T value);
    T pop();
    int size()
    {
        return size;
    }
    int capacity()
    {
        return capacity;
    }

private:
    T data[capacity];
    int headIndex;
    int size;
    inline int getRealIndex(int i)
    {
        return ((headIndex - index) % capacity + capacity) % capacity;
    }
};


/* read(int index)
 *
 * Returns the object stored at a given index.
 */
template <typename T, int capacity>
inline T RingBuffer<T, capacity>::read(int index)
{
    return data[getRealIndex(index)];
}


/* operator[int index]
 *
 * Returns a reference to the object stored at a given index.
 */
template <typename T, int capacity>
inline T& RingBuffer<T, capacity>::operator[](int index)
{
    return data[getRealIndex(index)];
}


/* write(T value, int index)
 *
 * Sets the value of the object stored at the given index.
 */
template <typename T, int capacity>
inline void RingBuffer<T, capacity>::write(T value, int index)
{
    data[getRealIndex(index)] = value;
}


/* push(T value)
 *
 * Pushes the given value to the head of the buffer.
 * Moves the head forward.
 * Overwrites the oldest value when size == capacity.
 */
template <typename T, int capacity>
inline void RingBuffer<T, capacity>::push(T value)
{
    ++headIndex %= capacity;
    data[headIndex] = value;
    if (size < capacity)
        ++size;
}


/* pop()
 *
 * Pops the top value off of the buffer and returns it.
 * Moves the head backward.
 * Calling pop() when size == 0 results in undefined behavior.
 */
template <typename T, int capacity>
inline T RingBuffer<T, capacity>::pop()
{
    int oldHead = headIndex;
    headIndex = (headIndex - 1 + capacity) % capacity;
    if (size > 0)
        --size;
    return data[oldHead]; 
}


#endif // RINGBUFFER_H