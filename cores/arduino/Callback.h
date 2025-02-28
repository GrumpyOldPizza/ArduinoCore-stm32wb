/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#pragma once

#include <cstddef>

class Callback {
public:
  // Callback() : m_callback(nullptr), m_context(nullptr) {  }
    Callback() = default;
    Callback(void (*function)(void)) : m_callback((void (*)(void*))function), m_context(nullptr) { }
    Callback(const Callback &other) : m_callback(other.m_callback), m_context(other.m_context) { }

    Callback &operator=(const Callback &other);

    template<typename T>
    Callback(void (T::*method)(), T *object) { bind(&method, object); }

    template<typename T>
    Callback(void (T::*method)() const, const T *object) { bind(&method, object); }

    template<typename T>
    Callback(void (T::*method)() volatile, volatile T *object) { bind(&method, object); }

    template<typename T>
    Callback(void (T::*method)() const volatile, const volatile T *object) { bind(&method, object); }

    template<typename T>
    Callback(void (T::*method)(), T &object) { bind(&method, &object); }

    template<typename T>
    Callback(void (T::*method)() const, const T &object) { bind(&method, &object); }

    template<typename T>
    Callback(void (T::*method)() volatile, volatile T &object) { bind(&method, &object); }

    template<typename T>
    Callback(void (T::*method)() const volatile, const volatile T &object) { bind(&method, &object); }

    void (*callback())(void*) { return m_callback; }
    void *context() { return m_context; }

    operator bool() { return (m_callback != nullptr); }

    void operator ()(void);

private:
    void (* volatile m_callback)(void*);
    void * volatile m_context;

    void bind(const void *method, const void *object);
};
