#include <iostream>
#include <webots/Robot.hpp>
#include "Python.h"


int main(int argc, char **argv)
{
    Py_Initialize();

    PyObject *sysPath = PySys_GetObject("path");
    PyList_Append(sysPath, PyUnicode_FromString(".."));

    PyObject *pName = PyUnicode_FromString("my_mod");
    PyObject *pModule = PyImport_Import(pName);

    std::cout << "Works fine till here\n";
    webots::Robot* robot = new webots::Robot();

    if (pModule != NULL)
    {
        std::cout << "Python module found\n";

        PyObject *pDict = PyModule_GetDict(pModule);
        PyObject *pClass = PyDict_GetItemString(pDict, "MyPlugin");
        if (!pClass) {
            std::cout << "Couldn't find func\n";
            return 1;
        }
        PyObject *pObject = PyObject_CallObject(pClass, nullptr);

        PyObject_CallMethod(pObject, "my_func", "");
    }
    else
        std::cout << "Python Module not found\n";

    return 0;
}
