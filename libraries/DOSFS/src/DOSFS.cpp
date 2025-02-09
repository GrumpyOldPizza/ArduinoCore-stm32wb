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

#include <Arduino.h>
#include "wiring_private.h"
#include "DOSFS.h"
#include "dosfs_api.h"

class FileInstance {
public:
    virtual ~FileInstance() = default;

    virtual void reference();
    virtual void unreference();

    virtual int availableForWrite();
    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *data, size_t size);
    virtual void flush();

    virtual int available();
    virtual int peek();
    virtual int read();
    virtual int read(uint8_t* data, size_t size);

    virtual bool seek(int32_t position, FileSeekMode mode);
    virtual int32_t position();
    virtual int32_t size();
    virtual void close();
    
    inline File file() { return File(this); }
    inline static FileInstance *instance(File &file) { return file.instance(); }
};

class DirInstance {
public:
    virtual ~DirInstance() = default;

    virtual void reference();
    virtual void unreference();

    virtual bool read(String &string, size_t &size, bool &isDirectory);
    virtual bool rewind();
    virtual void close();
    
    inline Dir dir() { return Dir(this); }
    inline static DirInstance *instance(Dir &dir) { return dir.instance(); }
};

void FileInstance::reference() {
}

void FileInstance::unreference() {
}

int FileInstance::availableForWrite() {
    return 0;
}

size_t FileInstance::write(uint8_t data __attribute__((unused))) {
    return 0;
}

size_t FileInstance::write(const uint8_t *data __attribute__((unused)), size_t size __attribute__((unused))) {
    return 0;
}

void FileInstance::flush() {
}

int FileInstance::available() {
    return 0;
}

int FileInstance::peek() {
    return -1;
}

int FileInstance::read() {
    return -1;
}

int FileInstance::read(uint8_t* data __attribute__((unused)), size_t size __attribute__((unused))) {
    return 0;
}

bool FileInstance::seek(int32_t position __attribute__((unused)), FileSeekMode mode __attribute__((unused))) {
    return false;
}

int32_t FileInstance::position() {
    return 0;
}

int32_t FileInstance::size() {
    return 0;
    
}

void FileInstance::close() {
    return;
}

void DirInstance::reference() {
}

void DirInstance::unreference() {
}

bool DirInstance::read(String &string __attribute__((unused)), size_t &size __attribute__((unused)), bool &isDirectory __attribute__((unused))) {
    return false;
}

bool DirInstance::rewind() {
    return false;
}

void DirInstance::close() {
}

FileInstance NullFile;
DirInstance NullDir;


class DOSFSFile : public FileInstance {
public:
    DOSFSFile();
    ~DOSFSFile() override;

    void reference() override;
    void unreference() override;

    int availableForWrite() override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *data, size_t size) override;
    void flush() override;

    int available() override;
    int peek() override;
    int read() override;
    int read(uint8_t* data, size_t size) override;

    bool seek(int32_t position, FileSeekMode mode) override;
    int32_t position() override;
    int32_t size() override;

    bool open(const char *path, const char *mode);
    void close() override;

private:
    volatile uint32_t m_refcount;

protected:
    F_FILE m_file;

    friend class DOSFSFileSystem;
};


class DOSFSDir : public DirInstance {
public:
    DOSFSDir();
    ~DOSFSDir() override;

    void reference() override;
    void unreference() override;

    bool read(String &string, size_t &size, bool &isDirectory) override;
    bool rewind() override;

    bool open(const char *path);
    void close() override;

private:
    volatile uint32_t m_refcount;

protected:
    F_DIR m_dir;

    friend class DOSFSFileSystem;
};


DOSFSFile::DOSFSFile() {
    m_refcount = 1;

    memset(&m_file, 0, sizeof(m_file));
}

DOSFSFile::~DOSFSFile() {
}

void DOSFSFile::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void DOSFSFile::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {

        f_close(&m_file);

        delete this;
    }
}

int DOSFSFile::availableForWrite() {
    int32_t position;

    position = f_tell(&m_file);

    if (position < 0) {
        return 0;
    }

    return ((uint32_t)0x80000000 - (uint32_t)position);
}

size_t DOSFSFile::write(uint8_t data) {
    return (f_putc(data, &m_file) == F_NO_ERROR) ? 1 : 0;
}

size_t DOSFSFile::write(const uint8_t *data, size_t size) {
    return f_write(data, 1, size, &m_file);
}

void DOSFSFile::flush() {
    f_flush(&m_file);
}

int DOSFSFile::available() {
    int32_t position, size;

    position = f_tell(&m_file);
    size = f_length(&m_file);

    if ((position < 0) || (size < 0)) {
        return 0;
    }

    return (size - position);
}

int DOSFSFile::peek() {
    int data;

    data = f_getc(&m_file);
    
    if (data < 0) {
        return -1;
    }

    f_seek(&m_file, -1, F_SEEK_CUR);
    
    return data;
}

int DOSFSFile::read() {
    int data;

    data = f_getc(&m_file);
    
    if (data < 0) {
        return -1;
    }
    
    return data;
}

int DOSFSFile::read(uint8_t* data, size_t size) {
    return f_read(data, 1, size, &m_file);
}

bool DOSFSFile::seek(int32_t position, FileSeekMode mode) {
    return (f_seek(&m_file, position, mode) == F_NO_ERROR);
}

int32_t DOSFSFile::position() {
    int32_t position;
    
    position = f_tell(&m_file);

    if (position < 0) {
        return -1;
    }

    return position;
}

int32_t DOSFSFile::size() {
    int32_t size;

    size = f_length(&m_file);

    if (size < 0) {
        return -1;
    }

    return size;
}

bool DOSFSFile::open(const char *path, const char *mode) {
    return (f_open(&m_file, path, mode) == F_NO_ERROR);
}

void DOSFSFile::close() {
    f_close(&m_file);
}


DOSFSDir::DOSFSDir() {
    m_refcount = 1;

    memset(&m_dir, 0, sizeof(m_dir));
}

DOSFSDir::~DOSFSDir() {
}

void DOSFSDir::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void DOSFSDir::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

bool DOSFSDir::read(String &string, size_t &size, bool &isDirectory) {
    F_DIRENT dirent;

    do {
        if (f_readdir(&m_dir, &dirent) != F_NO_ERROR) {
            return false;
        }
    } while (dirent.attr & (F_ATTR_HIDDEN | F_ATTR_SYSTEM | F_ATTR_VOLUME));
    
    string = String(dirent.filename);
    size = dirent.filesize;
    isDirectory = !!(dirent.attr & F_ATTR_DIR);

    return true;
}

bool DOSFSDir::rewind() {
    return (f_rewinddir(&m_dir) == F_NO_ERROR);
}    

bool DOSFSDir::open(const char *path) {
    return (f_opendir(&m_dir, path) == F_NO_ERROR);
}

void DOSFSDir::close() {
}

int DOSFSFileSystem::begin() {
#if (STM32WB_CONFIG_SFLASH == 1) || (STORAGE_TYPE == 1)
    dosfs_sflash_initialize();
#endif  

    return (f_initvolume() == F_NO_ERROR);
}

void DOSFSFileSystem::end() {
    f_delvolume();
}

bool DOSFSFileSystem::format() {
    return (f_format() == F_NO_ERROR);
}

bool DOSFSFileSystem::hardformat(size_t size) {
    return (f_hardformat(size) == F_NO_ERROR);
}

#if 0
bool DOSFSFileSystem::reclaim(size_t size) {
    return (f_reclaim(size) == F_NO_ERROR);
}
#endif

DOSFSFileSystem::operator bool() {
    return (f_checkvolume() == F_NO_ERROR);
}

File DOSFSFileSystem::open(const char *path, const char *mode) {
    DOSFSFile *file;

    file = new DOSFSFile();

    if (file) {
        if (file->open(path, mode)) {
            file->reference();

            return File(file->file());
        } else {
            delete file;
        }
    }
    
    return File();
}

Dir DOSFSFileSystem::openDir(const char *path) {
    DOSFSDir *dir;

    dir = new DOSFSDir();

    if (dir) {
        if (dir->open(path)) {
            dir->reference();

            return Dir(dir->dir());
        } else {
            delete dir;
        }
    }
    
    return Dir();
}

bool DOSFSFileSystem::exists(const char *path) {
    unsigned char attr;

    return (f_getattr(path, &attr) == F_NO_ERROR);
}

bool DOSFSFileSystem::remove(const char *path) {
    return (f_delete(path) == F_NO_ERROR);
}

bool DOSFSFileSystem::mkdir(const char *path) {
    return (f_mkdir(path) == F_NO_ERROR);
}

bool DOSFSFileSystem::rmdir(const char *path) {
    return (f_rmdir(path) == F_NO_ERROR);
}

DOSFSFileSystem DOSFS;


File::File() {
    m_instance = &NullFile;
}

File::~File() {
    (*m_instance).unreference();
}

File::File(const File &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

File &File::operator=(const File &other) {
    FileInstance *instance = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    instance->unreference();

    return *this;
}

File::File(File &&other) {
    m_instance = other.m_instance;

    other.m_instance = &NullFile;
}

File &File::operator=(File &&other) {
    FileInstance *instance = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &NullFile;

    instance->unreference();

    return *this;
}

File::File(FileInstance *instance) {
    m_instance = instance;

    instance->reference();
}

FileInstance *File::instance() {
    return m_instance;
}

File::operator bool() const {
    return m_instance != &NullFile;
}

int File::availableForWrite() {
    return (*m_instance).availableForWrite();
}

size_t File::write(uint8_t data) {
    return (*m_instance).write(data);
}

size_t File::write(const uint8_t *data, size_t size) {
    return (*m_instance).write(data, size);
}

void File::flush() {
    (*m_instance).flush();
}

int File::available() {
    return (*m_instance).available();
}

int File::peek() {
    return (*m_instance).peek();
}

int File::read() {
    return (*m_instance).read();
}

int File::read(uint8_t* data, size_t size) {
    return (*m_instance).read(data, size);
}

bool File::seek(int32_t position, FileSeekMode mode) {
    return (*m_instance).seek(position, mode);
}

int32_t File::position() {
    return (*m_instance).position();
}

int32_t File::size() {
    return (*m_instance).size();
}

void File::close() {
    (*m_instance).close();
}


Dir::Dir() {
    m_instance = &NullDir;
}

Dir::~Dir() {
    (*m_instance).unreference();
}

Dir::Dir(const Dir &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

Dir &Dir::operator=(const Dir &other) {
    DirInstance *instance = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    instance->unreference();

    return *this;
}

Dir::Dir(Dir &&other) {
    m_instance = other.m_instance;

    other.m_instance = &NullDir;
}

Dir &Dir::operator=(Dir &&other) {
    DirInstance *instance = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &NullDir;

    instance->unreference();

    return *this;
}

Dir::Dir(DirInstance *instance) {
    m_instance = instance;

    instance->reference();
}

DirInstance *Dir::instance() {
    return m_instance;
}

Dir::operator bool() const {
    return m_instance != &NullDir;
}

bool Dir::read(String &string, size_t &size, bool &isDirectory) {
    return (*m_instance).read(string, size, isDirectory);
}

bool Dir::rewind() {
    return (*m_instance).rewind();
}

void Dir::close() {
    (*m_instance).close();
}
