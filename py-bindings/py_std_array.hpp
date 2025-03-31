/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024,  
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
 * How to wrap c-array objects with boost.python and use them both from
 * C++ and Python.
 *
 * https://pyplusplus.readthedocs.io/en/latest/troubleshooting_guide/automatic_conversion/tuples.hpp.html
 * 
 *********************************************************************/


#ifndef PY_BINDINGS_PY_C_ARRAY_
#define PY_BINDINGS_PY_C_ARRAY_

#include <array>
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/python/object.hpp>
#include <boost/mpl/int.hpp>
#include <boost/mpl/next.hpp>

namespace boost {
namespace python {
namespace details {
  
//Small helper function, introduced to allow short syntax for index incrementing
template< int index>
typename mpl::next< mpl::int_< index > >::type increment_index(){
    typedef typename mpl::next< mpl::int_< index > >::type next_index_type;
    return next_index_type();
}

} // namespace details

template<typename TArray>
struct to_py_array {

    typedef mpl::int_<std::tuple_size<TArray>::value> length_type;

    static PyObject* convert(const TArray& c_array) {
        list values;
        //add all c_array items to "values" list
        convert_impl(c_array, values, mpl::int_<0>(), length_type());
        //create Python tuple from the list
        return incref(python::tuple(values).ptr());
    }

private:

    template<int index, int length>
    static void
    convert_impl(const TArray &c_array, list& values, mpl::int_<index>, mpl::int_<length>) {
        values.append(std::get<index>(c_array));
        convert_impl(c_array, values, details::increment_index<index>(), length_type());
    }

    template<int length>
    static void
    convert_impl(const TArray&, list&, mpl::int_<length>, mpl::int_<length>)
    {}

};

template<class TArray>
struct from_py_sequence {

    typedef TArray array_type;

    typedef mpl::int_<std::tuple_size<TArray>::value> length_type;

    static void*
    convertible(PyObject* py_obj){

        if (!PySequence_Check(py_obj)) {
            return 0;
        }

        if (!PyObject_HasAttrString(py_obj, "__len__")) {
            return 0;
        }

        python::object py_sequence(handle<>(borrowed(py_obj)));
    
        if (std::tuple_size<TArray>::value != len(py_sequence)) {
            return 0;
        }

        if (convertible_impl(py_sequence, mpl::int_<0>(), length_type())) {
            return py_obj;
        }
        else {
            return 0;
        }
    }

    static void
    construct(PyObject* py_obj, converter::rvalue_from_python_stage1_data* data) {
        typedef converter::rvalue_from_python_storage<TArray> storage_t;
        storage_t* the_storage = reinterpret_cast<storage_t*>(data);
        void* memory_chunk = the_storage->storage.bytes;
        TArray* c_array = new (memory_chunk)TArray();
        data->convertible = memory_chunk;

        python::object py_sequence(handle<>(borrowed(py_obj)));
        construct_impl(py_sequence, *c_array, mpl::int_<0>(), length_type());
    }

    static TArray to_c_array(PyObject* py_obj) {
        if (!convertible(py_obj)) {
            throw std::runtime_error("Unable to construct std::array from Python object!");
        }
        TArray c_array;
        python::object py_sequence(handle<>(borrowed(py_obj)));
        construct_impl(py_sequence, c_array, mpl::int_<0>(), length_type());
        return c_array;
    }

private:

    template<int index, int length>
    static bool
    convertible_impl(const python::object& py_sequence, mpl::int_<index>, mpl::int_<length>) {

        typedef typename std::tuple_element<index, TArray>::type element_type;

        object element = py_sequence[index];
        extract<element_type> type_checker(element);
        if (!type_checker.check()) {
            return false;
        }
        else {
            return convertible_impl(py_sequence, details::increment_index<index>(), length_type());
        }
    }

    template<int length>
    static bool
    convertible_impl(const python::object&, mpl::int_<length>, mpl::int_<length>) {
        return true;
    }

    template<int index, int length>
    static void
    construct_impl(const python::object& py_sequence, TArray& c_array, mpl::int_<index>, mpl::int_<length>) {

        typedef typename std::tuple_element<index, TArray>::type element_type;

        object element = py_sequence[index];
        get<index>(c_array) = extract<element_type>(element);

        construct_impl(py_sequence, c_array, details::increment_index<index>(), length_type());
    }

    template<int length>
    static void
    construct_impl(const python::object&, TArray&, mpl::int_<length>, mpl::int_<length>)
    {}

};

template< class TArray>
void register_array() {

    to_python_converter<TArray, to_py_array<TArray>>();

    converter::registry::push_back(
        &from_py_sequence<TArray>::convertible,
        &from_py_sequence<TArray>::construct,
        type_id<TArray>());
};


} // namespace python
} // namespace boost

#define PYREGISTER_ARRAY_3(T)                         \
    boost::python::register_array<std::array<T, 3> >();

#endif  // PY_BINDINGS_PY_C_ARRAY_
