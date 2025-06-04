// queue.h
#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>
#include "command.h" // for Cmd type or generic T works too

// Template circular queue with fixed capacity
// Note: implement entirely in header for templates

template <typename T>
class Queue {
public:
  // Constructor: capacity is maximum number of elements
  Queue(int capacity) : cap(capacity), head(0), tail(0), count(0) {
    buffer = (T*) malloc(sizeof(T) * cap);
  }
  ~Queue() {
    if (buffer) {
      free(buffer);
      buffer = nullptr;
    }
  }

  bool push(const T &item) {
    if (isFull()) return false;
    buffer[tail] = item;
    tail = (tail + 1) % cap;
    count++;
    return true;
  }

  T pop() {
    T val = buffer[head];
    head = (head + 1) % cap;
    count--;
    return val;
  }

  bool isEmpty() const {
    return (count == 0);
  }

  bool isFull() const {
    return (count == cap);
  }

  int size() const {
    return count;
  }

private:
  T *buffer;
  int head;
  int tail;
  int count;
  int cap;
};

#endif
