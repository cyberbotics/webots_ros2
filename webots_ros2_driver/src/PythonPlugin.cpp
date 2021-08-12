#include "webots_ros2_driver/PythonPlugin.hpp"

namespace webots_ros2_driver
{
    PythonPlugin::PythonPlugin(PyObject *pyPlugin) : mPyPlugin(pyPlugin){};

    void PythonPlugin::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
    {
        PyObject_CallMethod(mPyPlugin, "init", "");
    }

    void PythonPlugin::step()
    {
        PyObject_CallMethod(mPyPlugin, "step", "");
    }

    std::shared_ptr<PythonPlugin> PythonPlugin::createFromType(std::string &type)
    {
        const std::string moduleName = type.substr(0, type.find_last_of("."));
        const std::string className = type.substr(type.find_last_of(".") + 1);

        Py_Initialize();

        PyObject *pyName = PyUnicode_FromString(moduleName.c_str());
        PyObject *pyModule = PyImport_Import(pyName);
   
        // If the module cannot be find the error should be handled in the upper level (e.g. try loading C++ plugin)
        if (!pyModule)
            return NULL;

        PyObject *pyDict = PyModule_GetDict(pyModule);
        PyObject *pyClass = PyDict_GetItemString(pyDict, className.c_str());
        if (!pyClass)
            throw std::runtime_error("Error in plugin " + type + ": The class " + className + " cannot be found.");

        PyObject *pyPlugin = PyObject_CallObject(pyClass, nullptr);
        return std::shared_ptr<PythonPlugin>(new PythonPlugin(pyPlugin));
    }
}
