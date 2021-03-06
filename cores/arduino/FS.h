/*
 * Copyright (c) 2017-2021 Thomas Roell.  All rights reserved.
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

#ifndef FS_H
#define FS_H

#include <Arduino.h>

enum FileSeekMode {
    FILE_SEEK_SET = 0,
    FILE_SEEK_CUR = 1,
    FILE_SEEK_END = 2
};

class File : public Stream
{
public:
    File();
    ~File();

    File(const File &other);
    File &operator=(const File &other);
    File(File &&other);
    File &operator=(File &&other);

    operator bool() const;

    // Print methods:
    int availableForWrite() override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *data, size_t size) override;
    void flush() override;

    // Stream methods:
    int available() override;
    int peek() override;
    int read() override;
    int read(uint8_t* data, size_t size) override;

    // File methods
    bool seek(int32_t position, FileSeekMode mode = FILE_SEEK_SET);
    int32_t position();
    int32_t size();
    void close();

    using Print::write;

protected:
    File(class FileInstance *instance);
    class FileInstance *instance();

    friend class FileInstance;
    
private:
    FileInstance *m_instance;
};


class Dir 
{
public:
    Dir();
    ~Dir();

    Dir(const Dir &other);
    Dir &operator=(const Dir &other);
    Dir(Dir &&other);
    Dir &operator=(Dir &&other);

    operator bool() const;

    bool read(String &string, size_t &size, bool &isDirectory);
    bool rewind();
    void close();

protected:
    Dir(class DirInstance *instance);
    class DirInstance *instance();

    friend class DirInstance;
    
private:
    DirInstance *m_instance;
};

class FileSystem
{
public:
    virtual operator bool() = 0;

    virtual File open(const char *path, const char *mode) = 0;
    virtual Dir openDir(const char *path) = 0;
    virtual bool exists(const char* path) = 0;
    virtual bool remove(const char *path) = 0;
    virtual bool mkdir(const char *path) = 0;
    virtual bool rmdir(const char *path) = 0;

    File open(const String &path, const char *mode) { return open(path.c_str(), mode); }
    Dir openDir(const String &path) { return openDir(path.c_str()); }
    bool exists(const String& path) { return exists(path.c_str()); };
    bool remove(const String &path) { return remove(path.c_str()); }
    bool mkdir(const String &path) { return mkdir(path.c_str()); }
    bool rmdir(const String &path) { return rmdir(path.c_str()); }
};

#endif // FS_H
