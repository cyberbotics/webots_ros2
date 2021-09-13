#include "webots_ros2_driver/PythonPlugin.hpp"

static PyObject *gPyWebotsNode = NULL;

namespace webots_ros2_driver
{
    PythonPlugin::PythonPlugin(PyObject *pyPlugin) : mPyPlugin(pyPlugin){};

    void PythonPlugin::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
    {
        PyObject *pyParameters = PyDict_New();
        for (const std::pair<std::string, std::string> &parameter : parameters)
            PyDict_SetItem(pyParameters, PyUnicode_FromString(parameter.first.c_str()), PyUnicode_FromString(parameter.second.c_str()));

        PyObject_CallMethod(mPyPlugin, "init", "OO", getPyWebotsNodeInstance(), pyParameters);
        PyErr_Print();
    }

    void PythonPlugin::step()
    {
        PyObject_CallMethod(mPyPlugin, "step", "");
        PyErr_Print();
    }

    PyObject *PythonPlugin::getPyWebotsNodeInstance()
    {
        if (gPyWebotsNode)
            return gPyWebotsNode;

        PyObject *pyWebotsExtraModuleSource = Py_CompileString(
            R"EOT(
from webots_ros2_driver_webots.controller import Supervisor

class WebotsNode:
    def __init__(self):
        self.robot = Supervisor.internalGetInstance()
)EOT",
            "webots_extra", Py_file_input);
        if (!pyWebotsExtraModuleSource)
            throw std::runtime_error("Error: The Python module with the WebotsNode class cannot be compiled.");

        PyObject *pyWebotsExtraModule = PyImport_ExecCodeModule("webots_extra", pyWebotsExtraModuleSource);
        if (!pyWebotsExtraModule)
            throw std::runtime_error("Error: The Python module with the WebotsNode class cannot be executed.");

        PyObject *pyDict = PyModule_GetDict(pyWebotsExtraModule);
        PyObject *pyClass = PyDict_GetItemString(pyDict, "WebotsNode");
        gPyWebotsNode = PyObject_CallObject(pyClass, nullptr);
        return gPyWebotsNode;
    }

    std::shared_ptr<PythonPlugin> PythonPlugin::createFromType(const std::string &type)
    {
        const std::string moduleName = type.substr(0, type.find_last_of("."));
        const std::string className = type.substr(type.find_last_of(".") + 1);

        Py_Initialize();

        PyObject *pyName = PyUnicode_FromString(moduleName.c_str());
        PyObject *pyModule = PyImport_Import(pyName);
        PyErr_Print();

        // If the module cannot be found the error should be handled in the upper level (e.g. try loading C++ plugin)
        if (!pyModule)
            return NULL;

        PyObject *pyDict = PyModule_GetDict(pyModule);
        PyObject *pyClass = PyDict_GetItemString(pyDict, className.c_str());
        PyErr_Print();
        if (!pyClass)
            throw std::runtime_error("Error in plugin " + type + ": The class " + className + " cannot be found.");

        PyObject *pyPlugin = PyObject_CallObject(pyClass, nullptr);
        return std::shared_ptr<PythonPlugin>(new PythonPlugin(pyPlugin));
    }
}
