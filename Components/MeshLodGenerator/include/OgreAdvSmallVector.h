//
//  OgreAdvSmallVector.h
//  OgreMain
//
//  Created by Dmitry Yunchik on 11/16/18.
//  Copyright © 2018 Belight Software. All rights reserved.
//

#ifndef OgreAdvSmallVector_h
#define OgreAdvSmallVector_h

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef _WIN32
#define ASV_PACKED
#define ASV_ALIGN(N) __declspec(align(N))
#else
#define ASV_PACKED __attribute__((packed))
#define ASV_ALIGN(N) __attribute__((aligned(N)))
#endif

namespace Ogre {

namespace detail {

/*
 * Move objects in memory to the right into some uninitialized
 * memory, where the region overlaps.  This doesn't just use
 * std::move_backward because move_backward only works if all the
 * memory is initialized to type T already.
 */
template <class T>
typename std::enable_if<std::is_default_constructible<T>::value && !std::is_trivially_copyable<T>::value>::type
moveObjectsRight(T* first, T* lastConstructed, T* realLast)
{
    if (lastConstructed == realLast)
        return;

    T* end = first - 1; // Past the end going backwards.
    T* out = realLast - 1;
    T* in = lastConstructed - 1;
    try
    {
        for (; in != end && out >= lastConstructed; --in, --out)
        {
            new (out) T(std::move(*in));
        }
        for (; in != end; --in, --out)
        {
            *out = std::move(*in);
        }
        for (; out >= lastConstructed; --out)
        {
            new (out) T();
        }
    } catch (...)
    {
        // We want to make sure the same stuff is uninitialized memory
        // if we exit via an exception (this is to make sure we provide
        // the basic exception safety guarantee for insert functions).
        if (out < lastConstructed)
        {
            out = lastConstructed - 1;
        }
        for (auto it = out + 1; it != realLast; ++it)
        {
            it->~T();
        }
        throw;
    }
}

// Specialization for trivially copyable types.  The call to
// std::move_backward here will just turn into a memmove.  (TODO:
// change to std::is_trivially_copyable when that works.)
template <class T>
typename std::enable_if<!std::is_default_constructible<T>::value || std::is_trivially_copyable<T>::value>::type
moveObjectsRight(T* first, T* lastConstructed, T* realLast)
{
    std::move_backward(first, lastConstructed, realLast);
}

/*
 * Populate a region of memory using `op' to construct elements.  If
 * anything throws, undo what we did.
 */
template <class T, class Function>
void populateMemForward(T* mem, std::size_t n, Function const& op)
{
    std::size_t idx = 0;
    try
    {
        for (size_t i = 0; i < n; ++i)
        {
            op(&mem[idx]);
            ++idx;
        }
    } catch (...)
    {
        for (std::size_t i = 0; i < idx; ++i)
        {
            mem[i].~T();
        }
        throw;
    }
}

}//end_of_namespace detail

#ifdef _WIN32
#pragma pack(push, 1)
#endif

template <class Value, std::size_t InlineCapacity, typename SIZE_TYPE = unsigned int>
class ASV_ALIGN(sizeof(void*)) AdvSmallVector
{
public:
    typedef std::size_t size_type;
    typedef Value value_type;
    typedef value_type& reference;
    typedef value_type const& const_reference;
    typedef value_type* iterator;
    typedef value_type* pointer;
    typedef value_type const* const_iterator;
    typedef value_type const* const_pointer;
    typedef std::ptrdiff_t difference_type;
    typedef SIZE_TYPE SizeType;

    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
    
    static unsigned constexpr PtrAlign = sizeof(void*);
private:

    struct HeapWithCapacity {
        ASV_ALIGN(4) void* mHeap;
        SizeType mCapacity;

        SizeType getCapacity() const {
            return mCapacity;
        }
        void setCapacity(SizeType c) {
            mCapacity = c;
        }
    } ASV_PACKED ;
    

    union Data {
        explicit Data()
        {
            mHeapData.mHeap = nullptr;
        }

        HeapWithCapacity mHeapData;
        value_type mInlineStorage[InlineCapacity];

        value_type* buffer() noexcept { return &mInlineStorage[0]; }
        value_type const* buffer() const noexcept { return const_cast<Data*>(this)->buffer(); }
        value_type* heap() noexcept { return static_cast<value_type*>(mHeapData.mHeap); }
        value_type const* heap() const noexcept { return const_cast<Data*>(this)->heap(); }

        SizeType getCapacity() const { return mHeapData.getCapacity(); }
        void setCapacity(SizeType c) { mHeapData.setCapacity(c); }
        
        void freeHeap() { free(mHeapData.mHeap); }
        
    } ASV_PACKED mUnionData;

    SizeType mSize = 0;
    
    
    static SizeType constexpr kUseHeapMask = SizeType(1) << (sizeof(SizeType) * 8 - 1);
    
    static constexpr std::size_t maxSize() { return SizeType(~kUseHeapMask); }

    std::size_t getSize() const { return mSize & ~kUseHeapMask; }
    
    bool isUseHeap() const { return (kUseHeapMask & mSize)!=0; }
    bool isInline() const  { return (kUseHeapMask & mSize)==0; }

    void setUseHeap(bool b)
    {
        if (b)
        {
            mSize |= kUseHeapMask;
        } else
        {
            mSize &= ~kUseHeapMask;
        }
    }

    void setSize(std::size_t sz)
    {
        assert(sz <= maxSize());
        mSize = (kUseHeapMask & mSize) | SizeType(sz);
    }

    void swapSize(SizeType& o) { std::swap(mSize, o); }


    /*
    * Move a range to a range of uninitialized memory.  Assumes the
    * ranges don't overlap.
    */
    template <class T>
    typename std::enable_if<!std::is_trivially_copyable<T>::value>::type
    moveToUninitialized(T* first, T* last, T* out)
    {
        std::size_t idx = 0;
        try
        {
            for (; first != last; ++first, ++idx)
            {
                new (&out[idx]) T(std::move(*first));
            }
        } catch (...)
        {
            // Even for callers trying to give the strong guarantee
            // (e.g. push_back) it's ok to assume here that we don't have to
            // move things back and that it was a copy constructor that
            // threw: if someone throws from a move constructor the effects
            // are unspecified.
            for (std::size_t i = 0; i < idx; ++i)
            {
                out[i].~T();
            }
            throw;
        }
    }

    // Specialization for trivially copyable types.
    template <class T>
    typename std::enable_if<std::is_trivially_copyable<T>::value>::type
    moveToUninitialized(T* first, T* last, T* out)
    {
        std::memmove(out, first, (last - first) * sizeof *first);
    }

    /*
    * Move a range to a range of uninitialized memory. Assumes the
    * ranges don't overlap. Inserts an element at out + pos using
    * emplaceFunc(). out will contain (end - begin) + 1 elements on success and
    * none on failure. If emplaceFunc() throws [begin, end) is unmodified.
    */
    template <class T, class EmplaceFunc>
    void moveToUninitializedEmplace(T* begin, T* end, T* out, SizeType pos, EmplaceFunc&& emplaceFunc)
    {
        // Must be called first so that if it throws [begin, end) is unmodified.
        // We have to support the strong exception guarantee for emplace_back().
        emplaceFunc(out + pos);
        // move old elements to the left of the new one
        try
        {
            this->moveToUninitialized(begin, begin + pos, out);
        } catch (...)
        {
            out[pos].~T();
            throw;
        }
        // move old elements to the right of the new one
        try
        {
            if (begin + pos < end)
            {
                this->moveToUninitialized(begin + pos, end, out + pos + 1);
            }
        } catch (...)
        {
            for (SizeType i = 0; i <= pos; ++i)
            {
                out[i].~T();
            }
            throw;
        }
    }


public:

    AdvSmallVector() = default;
    // Allocator is unused here. It is taken in for compatibility with std::vector
    // interface, but it will be ignored.
    AdvSmallVector(const std::allocator<Value>&) {}

    AdvSmallVector(AdvSmallVector const& o)
    {
        auto n = o.size();
        makeSize(n);
        try
        {
            std::uninitialized_copy(o.begin(), o.end(), begin());
        } catch (...)
        {
            if (this->isUseHeap())
            {
                mUnionData.freeHeap();
            }
            throw;
        }
        this->setSize(n);
    }

    AdvSmallVector(AdvSmallVector&& o) noexcept(std::is_nothrow_move_constructible<Value>::value)
    {
        if (o.isInline())
        {
            std::uninitialized_copy(std::make_move_iterator(o.begin()), std::make_move_iterator(o.end()), begin());
            this->setSize(o.size());
        } else
        {
            swap(o);
        }
    }

    AdvSmallVector(std::initializer_list<value_type> il)
    {
        constructImpl(il.begin(), il.end(), std::false_type());
    }

    explicit AdvSmallVector(size_type n)
    {
        doConstruct(n, [&](void* p) { new (p) value_type(); });
    }

    AdvSmallVector(size_type n, value_type const& t)
    {
        doConstruct(n, [&](void* p) { new (p) value_type(t); });
    }

    template <class Arg>
    explicit AdvSmallVector(Arg arg1, Arg arg2)
    {
        // Forward using std::is_arithmetic to get to the proper
        // implementation; this disambiguates between the iterators and
        // (size_t, value_type) meaning for this constructor.
        constructImpl(arg1, arg2, std::is_arithmetic<Arg>());
    }

    ~AdvSmallVector()
    {
        for (auto& t : *this)
        {
            (&t)->~value_type();
        }
        if (this->isUseHeap())
        {
            mUnionData.freeHeap();
        }
    }

    AdvSmallVector& operator=(AdvSmallVector const& o)
    {
        if (this != &o)
        {
            assign(o.begin(), o.end());
        }
        return *this;
    }

    AdvSmallVector& operator=(AdvSmallVector&& o)
    {
        // TODO: optimization:
        // if both are internal, use move assignment where possible
        if (this != &o)
        {
            clear();
            swap(o);
        }
        return *this;
    }

    bool operator==(AdvSmallVector const& o) const
    {
        return size() == o.size() && std::equal(begin(), end(), o.begin());
    }

    bool operator<(AdvSmallVector const& o) const
    {
        return std::lexicographical_compare(begin(), end(), o.begin(), o.end());
    }

    static constexpr size_type max_size()
    {
        return SizeType(1) << (sizeof(SizeType) * 8 - 1);
    }

    size_type size() const { return this->getSize(); }
    bool empty() const { return !size(); }

    value_type* data() noexcept { return this->isInline() ? mUnionData.buffer() : mUnionData.heap(); }
    value_type const* data() const noexcept { return this->isInline() ? mUnionData.buffer() : mUnionData.heap(); }

    iterator begin()                { return data(); }
    iterator end()                  { return data() + size(); }
    const_iterator begin() const    { return data(); }
    const_iterator end() const      { return data() + size(); }
    const_iterator cbegin() const   { return begin(); }
    const_iterator cend() const     { return end(); }
    reverse_iterator rbegin()               { return reverse_iterator(end()); }
    reverse_iterator rend()                 { return reverse_iterator(begin()); }
    const_reverse_iterator rbegin() const   { return const_reverse_iterator(end()); }
    const_reverse_iterator rend() const     { return const_reverse_iterator(begin()); }
    const_reverse_iterator crbegin() const  { return rbegin(); }
    const_reverse_iterator crend() const    { return rend(); }

    /*
    * Usually one of the simplest functions in a Container-like class
    * but a bit more complex here.  We have to handle all combinations
    * of in-place vs. heap between this and o.
    *
    * Basic guarantee only.  Provides the nothrow guarantee iff our
    * value_type has a nothrow move or copy constructor.
    */
    void swap(AdvSmallVector& o)
    {
        using std::swap;

        if (this->isUseHeap() && o.isUseHeap())
        {
            swapSize(o.mSize);

            auto thisCapacity = this->capacity();
            auto oCapacity = o.capacity();

            auto* tmp = mUnionData.mHeapData.mHeap;
            mUnionData.mHeapData.mHeap = o.mUnionData.mHeapData.mHeap;
            o.mUnionData.mHeapData.mHeap = tmp;

            this->setCapacity(oCapacity);
            o.setCapacity(thisCapacity);

            return;
        }

        if (this->isInline() && o.isInline())
        {
            auto& oldSmall = size() < o.size() ? *this : o;
            auto& oldLarge = size() < o.size() ? o : *this;

            for (size_type i = 0; i < oldSmall.size(); ++i)
            {
                swap(oldSmall[i], oldLarge[i]);
            }

            size_type i = oldSmall.size();
            const size_type ci = i;
            try
            {
                for (; i < oldLarge.size(); ++i)
                {
                    auto addr = oldSmall.begin() + i;
                    new (addr) value_type(std::move(oldLarge[i]));
                    oldLarge[i].~value_type();
                }
            } catch (...)
            {
                oldSmall.setSize(i);
                for (; i < oldLarge.size(); ++i)
                {
                    oldLarge[i].~value_type();
                }
                oldLarge.setSize(ci);
                throw;
            }
            oldSmall.setSize(i);
            oldLarge.setSize(ci);
            return;
        }

        // isInline != o.isInline()
        auto& oldExtern = o.isInline() ? *this : o;
        auto& oldIntern = o.isInline() ? o : *this;

        auto oldExternCapacity = oldExtern.capacity();
        auto oldExternHeap = oldExtern.mUnionData.mHeapData.mHeap;

        auto buff = oldExtern.mUnionData.buffer();
        size_type i = 0;
        try
        {
            for (; i < oldIntern.size(); ++i)
            {
                new (&buff[i]) value_type(std::move(oldIntern[i]));
                oldIntern[i].~value_type();
            }
        } catch (...)
        {
            for (size_type kill = 0; kill < i; ++kill)
            {
                buff[kill].~value_type();
            }
            for (; i < oldIntern.size(); ++i)
            {
                oldIntern[i].~value_type();
            }
            oldIntern.setSize(0);
            oldExtern.mUnionData.mHeapData.mHeap = oldExternHeap;
            oldExtern.setCapacity(oldExternCapacity);
            throw;
        }
        oldIntern.mUnionData.mHeapData.mHeap = oldExternHeap;
        swapSize(o.mSize);
        oldIntern.setCapacity(oldExternCapacity);
    }

    void resize(size_type sz)
    {
        if (sz < size())
        {
            erase(begin() + sz, end());
            return;
        }
        makeSize(sz);
        detail::populateMemForward(begin() + size(), sz - size(), [&](void* p) { new (p) value_type(); });
        this->setSize(sz);
    }

    void resize(size_type sz, value_type const& v)
    {
        if (sz < size())
        {
            erase(begin() + sz, end());
            return;
        }
        makeSize(sz);
        detail::populateMemForward(begin() + size(), sz - size(), [&](void* p) { new (p) value_type(v); });
        this->setSize(sz);
    }

    template <class... Args>
    iterator emplace(const_iterator p, Args&&... args)
    {
        if (p == cend())
        {
            emplace_back(std::forward<Args>(args)...);
            return end() - 1;
        }

        /*
         * We implement emplace at places other than at the back with a
         * temporary for exception safety reasons.  It is possible to
         * avoid having to do this, but it becomes hard to maintain the
         * basic exception safety guarantee (unless you respond to a copy
         * constructor throwing by clearing the whole vector).
         *
         * The reason for this is that otherwise you have to destruct an
         * element before constructing this one in its place---if the
         * constructor throws, you either need a nothrow default
         * constructor or a nothrow copy/move to get something back in the
         * "gap", and the vector requirements don't guarantee we have any
         * of these.  Clearing the whole vector is a legal response in
         * this situation, but it seems like this implementation is easy
         * enough and probably better.
         */
        return insert(p, value_type(std::forward<Args>(args)...));
    }

    void reserve(size_type sz)
    {
        makeSize(sz);
    }

    size_type capacity() const
    {
        if (this->isUseHeap())
        {
            return mUnionData.getCapacity();
        }
        return InlineCapacity;
    }

    void shrink_to_fit()
    {
        if (this->isInline())
            return;

        AdvSmallVector tmp(begin(), end());
        tmp.swap(*this);
    }

    template <class... Args>
    void emplace_back(Args&&... args)
    {
        if (capacity() == size())
        {
            // Any of args may be references into the vector.
            // When we are reallocating, we have to be careful to construct the new
            // element before modifying the data in the old buffer.
            makeSize(size() + 1, [&](void* p) { new (p) value_type(std::forward<Args>(args)...); }, size());
        } else
        {
            new (end()) value_type(std::forward<Args>(args)...);
        }
        this->setSize(size() + 1);
    }

    void push_back(value_type&& t)
    {
        return emplace_back(std::move(t));
    }

    void push_back(const value_type& t)
    {
        emplace_back(t);
    }

    void pop_back()
    {
        erase(end() - 1);
    }

    iterator insert(const_iterator constp, value_type&& t)
    {
        iterator p = unconst(constp);

        if (p == end())
        {
            push_back(std::move(t));
            return end() - 1;
        }

        auto offset = p - begin();

        if (capacity() == size())
        {
            makeSize(size() + 1, [&t](void* ptr) { new (ptr) value_type(std::move(t)); }, offset);
            this->setSize(this->size() + 1);
        } else
        {
            detail::moveObjectsRight(data() + offset, data() + size(), data() + size() + 1);
            this->setSize(size() + 1);
            data()[offset] = std::move(t);
        }
        
        return begin() + offset;
    }

    iterator insert(const_iterator p, value_type const& t)
    {
        // Make a copy and forward to the rvalue value_type&& overload above.
        return insert(p, (value_type&&)value_type(t));
    }

    iterator insert(const_iterator pos, size_type n, value_type const& val)
    {
        auto offset = pos - begin();
        makeSize(size() + n);
        detail::moveObjectsRight(data() + offset, data() + size(), data() + size() + n);
        this->setSize(size() + n);
        std::generate_n(begin() + offset, n, [&] { return val; });
        return begin() + offset;
    }

    template <class Arg>
    iterator insert(const_iterator p, Arg arg1, Arg arg2)
    {
        // Forward using std::is_arithmetic to get to the proper
        // implementation; this disambiguates between the iterators and
        // (size_t, value_type) meaning for this function.
        return insertImpl(unconst(p), arg1, arg2, std::is_arithmetic<Arg>());
    }

    iterator insert(const_iterator p, std::initializer_list<value_type> il)
    {
        return insert(p, il.begin(), il.end());
    }

    iterator erase(const_iterator q)
    {
        std::move(unconst(q) + 1, end(), unconst(q));
        (data() + size() - 1)->~value_type();
        this->setSize(size() - 1);
        return unconst(q);
    }

    iterator erase(const_iterator q1, const_iterator q2)
    {
        if (q1 == q2)
            return unconst(q1);
        
        std::move(unconst(q2), end(), unconst(q1));
        for (auto it = (end() - std::distance(q1, q2)); it != end(); ++it)
        {
            it->~value_type();
        }
        
        this->setSize(size() - (q2 - q1));
        return unconst(q1);
    }

    void clear()
    {
        erase(begin(), end());
    }

    template <class Arg>
    void assign(Arg first, Arg last)
    {
        clear();
        insert(end(), first, last);
    }

    void assign(std::initializer_list<value_type> il)
    {
        assign(il.begin(), il.end());
    }

    void assign(size_type n, const value_type& t)
    {
        clear();
        insert(end(), n, t);
    }

    reference front()
    {
        assert(!empty());
        return *begin();
    }
    reference back()
    {
        assert(!empty());
        return *(end() - 1);
    }
    const_reference front() const
    {
        assert(!empty());
        return *begin();
    }
    const_reference back() const
    {
        assert(!empty());
        return *(end() - 1);
    }

    reference operator[](size_type i)
    {
        assert(i < size());
        return *(begin() + i);
    }

    const_reference operator[](size_type i) const
    {
        assert(i < size());
        return *(begin() + i);
    }

    reference at(size_type i)
    {
        if (i >= size())
        {
            throw std::out_of_range("index out of range");
        }
        return (*this)[i];
    }

    const_reference at(size_type i) const
    {
        if (i >= size())
        {
            throw std::out_of_range("index out of range");
        }
        return (*this)[i];
    }

private:

    static iterator unconst(const_iterator it)
    {
        return const_cast<iterator>(it);
    }

    // The std::false_type argument is part of disambiguating the
    // iterator insert functions from integral types (see insert().)
    template <class It>
    iterator insertImpl(iterator pos, It first, It last, std::false_type)
    {
        typedef typename std::iterator_traits<It>::iterator_category categ;
        if (std::is_same<categ, std::input_iterator_tag>::value)
        {
            auto offset = pos - begin();
            while (first != last)
            {
                pos = insert(pos, *first++);
                ++pos;
            }
            return begin() + offset;
        }

        auto distance = std::distance(first, last);
        auto offset = pos - begin();
        makeSize(size() + distance);
        detail::moveObjectsRight(data() + offset, data() + size(), data() + size() + distance);
        this->setSize(size() + distance);
        std::copy_n(first, distance, begin() + offset);
        return begin() + offset;
    }

    iterator insertImpl(iterator pos, size_type n, const value_type& val, std::true_type)
    {
        // The true_type means this should call the size_t,value_type
        // overload.  (See insert().)
        return insert(pos, n, val);
    }

    // The std::false_type argument came from std::is_arithmetic as part
    // of disambiguating an overload (see the comment in the
    // constructor).
    template <class It>
    void constructImpl(It first, It last, std::false_type)
    {
        typedef typename std::iterator_traits<It>::iterator_category categ;
        if (std::is_same<categ, std::input_iterator_tag>::value)
        {
            // With iterators that only allow a single pass, we can't really
            // do anything sane here.
            while (first != last)
            {
                emplace_back(*first++);
            }
            return;
        }

        auto distance = std::distance(first, last);
        makeSize(distance);
        this->setSize(distance);
        try
        {
            detail::populateMemForward(data(), distance, [&](void* p) { new (p) value_type(*first++); });
        } catch (...)
        {
            if (this->isUseHeap())
            {
                mUnionData.freeHeap();
            }
            throw;
        }
    }

    template <typename InitFunc>
    void doConstruct(size_type n, InitFunc&& func)
    {
        makeSize(n);
        this->setSize(n);
        try
        {
            detail::populateMemForward(data(), n, std::forward<InitFunc>(func));
        } catch (...)
        {
            if (this->isUseHeap())
            {
                mUnionData.freeHeap();
            }
            throw;
        }
    }

    // The true_type means we should forward to the size_t,value_type
    // overload.
    void constructImpl(size_type n, value_type const& val, std::true_type)
    {
        doConstruct(n, [&](void* p) { new (p) value_type(val); });
    }

    /*
    * Compute the size after growth.
    */
    size_type computeNewSize() const
    {
        return std::min((3 * capacity()) / 2 + 1, max_size());
    }

    void makeSize(size_type newSize)
    {
        makeSizeInternal(newSize, false, [](void*) { assert(false);/*assume_unreachable();*/ }, 0);
    }

    template <typename EmplaceFunc>
    void makeSize(size_type newSize, EmplaceFunc&& emplaceFunc, size_type pos)
    {
        assert(size() == capacity());
        makeSizeInternal(newSize, true, std::forward<EmplaceFunc>(emplaceFunc), pos);
    }

    /*
    * Ensure we have a large enough memory region to be size `newSize'.
    * Will move/copy elements if we are spilling to heap or needed to
    * allocate a new region, but if resized in place doesn't initialize
    * anything in the new region.  In any case doesn't change size().
    * Supports insertion of new element during reallocation by given
    * pointer to new element and position of new element.
    * NOTE: If reallocation is not needed, insert must be false,
    * because we only know how to emplace elements into new memory.
    */
    template <typename EmplaceFunc>
    void makeSizeInternal(size_type newSize, bool insert, EmplaceFunc&& emplaceFunc, size_type pos)
    {
        if (newSize > max_size())
        {
            throw std::length_error("max_size exceeded in AdvSmallVector");
        }
        if (newSize <= capacity())
        {
            assert(!insert);
            return;
        }

        newSize = std::max(newSize, computeNewSize());

        auto needBytes = newSize * sizeof(value_type);
        auto const sizeBytes = needBytes; //goodMallocSize(needBytes);
        void* newh = malloc(sizeBytes); //checkedMalloc(sizeBytes);
        if(!newh)
        {
            throw std::bad_alloc();
        }
        
        value_type* newp = (value_type*)newh;

        try
        {
            if (insert)
            {
                // move and insert the new element
                this->moveToUninitializedEmplace(begin(), end(), newp, (SizeType)pos, std::forward<EmplaceFunc>(emplaceFunc));
            } else
            {
                // move without inserting new element
                this->moveToUninitialized(begin(), end(), newp);
            }
        } catch (...)
        {
            free(newh);
            throw;
        }
        
        for (auto& val : *this)
        {
            val.~value_type();
        }

        if (this->isUseHeap())
        {
            mUnionData.freeHeap();
        }
        mUnionData.mHeapData.mHeap = newh;
        this->setUseHeap(true);
        this->setCapacity(sizeBytes / sizeof(value_type));
    }

    /*
    * This will set the capacity field, stored inline in the storage_ field
    * if there is sufficient room to store it.
    */
    void setCapacity(size_type newCapacity)
    {
        assert(this->isUseHeap());
        assert(newCapacity < std::numeric_limits<SizeType>::max());
        mUnionData.setCapacity((SizeType)newCapacity);
    }

};

#ifdef _WIN32
#pragma pack(pop)
#endif

// Basic guarantee only, or provides the nothrow guarantee iff T has a
// nothrow move or copy constructor.
template <class Value, std::size_t InlineCapacity>
void swap(
    AdvSmallVector<Value, InlineCapacity>& a,
    AdvSmallVector<Value, InlineCapacity>& b)
{
    a.swap(b);
}


}


#endif /* OgreAdvSmallVector_h */
