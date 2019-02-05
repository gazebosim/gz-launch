/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <list>
#include <mutex>
#include <numeric>
#include <thread>
#include <condition_variable>
#include <unordered_set>

#include <unistd.h>
#include <signal.h>
#include <tinyxml2.h>

#include <sys/wait.h>

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/plugin/Loader.hh>

#include "ignition/launch/config.hh"
#include "ignition/launch/Plugin.hh"
#include "Manager.hh"

using namespace ignition;
using namespace launch;

/// \brief A class to encapsulate an executable (program) to run.
class Executable
{
  /// \brief Default constructor
  public: Executable() = default;

  /// \brief Constructs a executable object with a name, process id, command,
  /// and other options. This class is used primarily for internal
  /// book keeping.
  /// \param[in] _name Name of the executable. This is used for internal
  /// book keeping, it is not (necessarily) the name of the executable that
  /// will be run.
  /// \param[in] _pid The pid in which the executable is run.
  /// \param[in] _cmd The command and arguments that specify how to run the
  /// executable.
  /// \param[in] _autoRestart True if the executable should restart when it
  /// is is killed.
  public: Executable(const std::string &_name, const pid_t _pid,
            const std::vector<std::string> &_cmd, bool _autoRestart,
            const std::list<std::pair<std::string, std::string>> _envs)
          : name(_name), pid(_pid), command(_cmd), autoRestart(_autoRestart),
            envs(_envs)
          {}

  /// \brief Name of the executable
  public: std::string name = "";

  /// \brief Process id in which the executable is run.
  public: pid_t pid = -1;

  /// \brief The command to run.
  public: std::vector<std::string> command;

  /// \brief True will cause the command to restart on kill
  public: bool autoRestart = false;

  /// \brief Environment variables.
  public: std::list<std::pair<std::string, std::string>> envs;
};

/// \brief Private data variables for the Ignition class.
class ignition::launch::ManagerPrivate
{
  /// \brief Constructor.
  public: ManagerPrivate();

  /// \brief Parse a configuration file.
  /// \param[in] _filename Name of the XML file to parse.
  /// \return True on success.
  public: bool ParseConfig(const std::string &_filename);

  /// \brief Fork a new process for a command specified by _exec. This
  /// function will create a new Executable and add it to the executables
  /// list.
  /// \param[in] _exec An executable definition to run.
  /// \return True on success.
  public: bool RunExecutable(const Executable &_exec);

  /// \brief Fork a new process for a command specific by _cmd.
  /// \param[in] _name A unique name given to the command. This name is
  /// used for book keeping and control of the process.
  /// \param[in] _cmd A vector of strings where the first string is
  /// expected to be the name of the executable to run, and subsequent
  /// strings are arguments.
  /// \param[in] _autoRestart True if the executable should auto restart on
  /// death.
  /// \return True on success.
  public: bool RunExecutable(const std::string &_name,
    const std::vector<std::string> &_cmd,
    const bool _autoRestart,
    const std::list<std::pair<std::string, std::string>> _envs);

  /// \brief Stop all executables
  public: void ShutdownExecutables();

  /// \brief Stop running. This can be used to end a Run().
  /// \return True if running, False if the state was not running.
  public: bool Stop();

  /// \brief Print help information to stdout.
  public: void PrintUsage();

  public: void LoadPlugin(const std::string &_filename,
              const tinyxml2::XMLElement *_elem);

  /// \brief Handle SIG_INT and SIG_TERM signals
  /// \param[in] _sig The signal
  private: void OnSigIntTerm(int _sig);

  /// \brief Handle SIG_CHILD
  /// \param[in] _sig The signal
  private: static void OnSigChild(int _sig);

  /// \brief A list of executables that are running, or have been run.
  public: std::list<Executable> executables;
  public: std::unordered_set<launch::PluginPtr> plugins;

  /// \brief Mutex to protect the executables list.
  public: std::mutex executablesMutex;

  /// \brief A condition variable used by SIG_INT and Manager::Run.
  public: std::condition_variable runCondition;

  /// \brief A mutex used in conjunction with runCondition.
  public: std::mutex runMutex;

  /// \brief True if running.
  public: bool running = false;

  public: bool master = false;

  /// \brief Our signal handler.
  public: common::SignalHandler sigHandler;

  /// \brief Pointer to myself. This is used in the signal handlers.
  /// A raw pointer is acceptable here since it is used only internally.
  public: static ManagerPrivate *myself;
};

ManagerPrivate *ManagerPrivate::myself = nullptr;

/////////////////////////////////////////////////
Manager::Manager()
  :dataPtr(new ManagerPrivate())
{
  this->dataPtr->myself = this->dataPtr.get();

  // Make sure to initialize logging.
  ignLogInit("~/.ignition", "launch.log");
  if (!this->dataPtr->sigHandler.Initialized())
    ignerr << "signal(2) failed while setting up for SIGINT" << std::endl;
}

/////////////////////////////////////////////////
Manager::~Manager()
{
}

/////////////////////////////////////////////////
bool Manager::RunConfig(const std::string &_config)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->runMutex);
  this->dataPtr->master = true;

  this->dataPtr->ParseConfig(_config);

  // Child processes should exit
  if (!this->dataPtr->master)
    return true;

  this->dataPtr->running = !this->dataPtr->executables.empty() ||
                           !this->dataPtr->plugins.empty();

  // Wait for a shutdown event.
  // \todo: In the future, we could add execution models that control plugin
  // updates.
  while (this->dataPtr->running)
    this->dataPtr->runCondition.wait(lock);

  // Stop executables.
  this->dataPtr->ShutdownExecutables();

  // Stop plugins.
  std::cout << "1\n";
  this->dataPtr->plugins.clear();
  std::cout << "2\n";

  return true;
}

/////////////////////////////////////////////////
bool Manager::Stop()
{
  return this->dataPtr->Stop();
}

/////////////////////////////////////////////////
ManagerPrivate::ManagerPrivate()
{
  this->sigHandler.AddCallback(
      std::bind(&ManagerPrivate::OnSigIntTerm, this, std::placeholders::_1));

  // Register a signal handler to capture child process death events.
  if (signal(SIGCHLD, ManagerPrivate::OnSigChild) == SIG_ERR)
    ignerr << "signal(2) failed while setting up for SIGCHLD" << std::endl;
}

/////////////////////////////////////////////////
bool ManagerPrivate::Stop()
{
  std::lock_guard<std::mutex> lock(this->runMutex);

  if (this->running)
  {
    this->running = false;
    this->runCondition.notify_all();
  }

  return this->running;
}

/////////////////////////////////////////////////
void ManagerPrivate::PrintUsage()
{
  std::cerr << "ignition -- Run the Ignition program and plugin manager.\n\n";
  std::cerr << "`ignition` [options]\n\n";
  std::cerr << "Ignition launches the specified programs and/or plugins.\n\n";
}

/////////////////////////////////////////////////
void ManagerPrivate::OnSigIntTerm(int _sig)
{
  igndbg << "Received signal[" << _sig  << "]\n";
  myself->Stop();
}

/////////////////////////////////////////////////
void ManagerPrivate::OnSigChild(int _sig)
{
  igndbg << "Received signal[" << _sig  << "]\n";
  pid_t p;
  int status;

  // Executable to restart
  Executable restartExec;

  if ((p = waitpid(-1, &status, WNOHANG)) != -1)
  {
    std::lock_guard<std::mutex> mutex(myself->executablesMutex);

    // Find the executable
    for (std::list<Executable>::iterator iter = myself->executables.begin();
         iter != myself->executables.end(); ++iter)
    {
      if (iter->pid == p)
      {
        igndbg << "Death of process[" << p << "] with name["
               << iter->name << "].\n";

        // Restart if autoRestart is enabled
        if (iter->autoRestart)
        {
          restartExec = *iter;
          myself->executables.erase(iter);
        }
        break;
      }
    }
  }

  if (!restartExec.name.empty() && !restartExec.command.empty())
  {
    igndbg << "Restarting process with name[" << restartExec.name << "]\n";
    myself->RunExecutable(restartExec);
  }
}

/////////////////////////////////////////////////
bool ManagerPrivate::ParseConfig(const std::string &_filename)
{
  tinyxml2::XMLDocument xmlDoc;

  // Load the XML configuration file into TinyXML
  if (xmlDoc.LoadFile(_filename.c_str()) != tinyxml2::XML_SUCCESS)
  {
    ignerr << "Unable to load config file[" << _filename << "]\n";
    return false;
  }

  // Get the root element.
  tinyxml2::XMLElement *root = xmlDoc.FirstChildElement("ignition");
  if (!root)
  {
    ignerr << "Invalid config file[" << _filename << "]. "
      << "Missing <ignition> element\n";
    return false;
  }

  // Process all the executables.
  tinyxml2::XMLElement *execElem = root->FirstChildElement("executable");

  // This "i" variable is just used for output messages.
  for (int i = 0; execElem; ++i)
  {
    bool autoRestart = false;
    bool valid = true;
    std::vector<std::string> cmdParts;

    // Get the executable's name
    std::string nameStr = execElem->Attribute("name");
    if (nameStr.empty())
    {
      valid = false;
      ignerr << "Invalid configuration file[" << _filename << "]. "
        << "Missing name attribute for the " << i << " <executable> element."
        << std::endl;
    }

    // Get the command
    tinyxml2::XMLElement *cmdElem = execElem->FirstChildElement("command");
    if (!cmdElem)
    {
      valid = false;
      ignerr << "Invalid configuration file[" << _filename << "]. "
        << " Missing <command> child element "
        << " of <executable name=\"" << nameStr << "\">\n";
    }
    else
    {
      std::vector<std::string> parts =
        ignition::common::split(cmdElem->GetText(), " ");
      std::move(parts.begin(), parts.end(), std::back_inserter(cmdParts));
    }

    // Get the <auto_restart> element. It is okay if the <auto_restart> element
    // is not present
    tinyxml2::XMLElement *restartElem = execElem->FirstChildElement(
        "auto_restart");
    if (restartElem && restartElem->GetText())
    {
      std::string txt = ignition::common::lowercase(restartElem->GetText());
      autoRestart = txt == "true" || txt == "1" || txt == "t";
    }

    std::list<std::pair<std::string, std::string>> envs;

    // Get the environment variables
    tinyxml2::XMLElement *envElem = execElem->FirstChildElement("env");
    while (envElem)
    {
      tinyxml2::XMLElement *nameElem = envElem->FirstChildElement("name");
      tinyxml2::XMLElement *valueElem = envElem->FirstChildElement("value");
      if (nameElem && valueElem)
      {
        std::string name = nameElem->GetText();
        std::string value = valueElem->GetText();
        envs.push_back({name, value});
      }

      envElem = envElem->NextSiblingElement("env");
    }

    if (valid)
    {
      if (!this->RunExecutable(nameStr, cmdParts, autoRestart, envs))
      {
        ignerr << "Unable to run executable named[" << nameStr << "] in "
          << "configuration file[" << _filename << "].\n";
      }
    }

    execElem = execElem->NextSiblingElement("executable");
  }

  // Process all the plugins.
  tinyxml2::XMLElement *pluginElem = root->FirstChildElement("plugin");


  // This "i" variable is just used for output messages.
  while (pluginElem)
  {
    this->LoadPlugin(_filename, pluginElem);
    pluginElem = pluginElem->NextSiblingElement("plugin");
  }

  return true;
}

/////////////////////////////////////////////////
bool ManagerPrivate::RunExecutable(const Executable &_exec)
{
  return this->RunExecutable(_exec.name, _exec.command, _exec.autoRestart,
      _exec.envs);
}

/////////////////////////////////////////////////
bool ManagerPrivate::RunExecutable(const std::string &_name,
    const std::vector<std::string> &_cmd, bool _autoRestart,
    const std::list<std::pair<std::string, std::string>> _envs)
{
  // Check for empty
  if (_cmd.empty())
  {
    ignerr << "Empty command.\n";
    return false;
  }

  // Fork a process for the command
  pid_t pid = fork();

  // If parent process...
  if (pid)
  {
    igndbg << "Forked a process for [" << _name << "] command["
      << std::accumulate(_cmd.begin(), _cmd.end(), std::string("")) << "]\n"
      << std::flush;

    std::lock_guard<std::mutex> mutex(this->executablesMutex);
    // Store the PID in the parent process.
    this->executables.push_back(Executable(
          _name, pid, _cmd, _autoRestart, _envs));
  }
  // Else child process...
  else
  {
    // A child is not the master
    this->master = false;

    // Create a vector of char* in the child process
    std::vector<char*> cstrings;
    for (const std::string &part : _cmd)
    {
      cstrings.push_back(const_cast<char *>(part.c_str()));
    }

    // Add the nullptr termination.
    cstrings.push_back(nullptr);

    // Remove from foreground process group.
    setpgid(0, 0);

    for (const std::pair<std::string, std::string> &env : _envs)
    {
      char finalEnv[1024];
      snprintf(finalEnv, sizeof(finalEnv), "%s=%s", env.first.c_str(),
          env.second.c_str());
      putenv(finalEnv);
    }

    // Run the command
    if (execvp(cstrings[0], &cstrings[0]) < 0)
    {
      ignerr << "Unable to run command["
        << std::accumulate(_cmd.begin(), _cmd.end(), std::string("")) << "]\n";
    }
  }

  return true;
}

/////////////////////////////////////////////////
void ManagerPrivate::ShutdownExecutables()
{
  std::lock_guard<std::mutex> mutex(this->executablesMutex);

  // Remove the sigchld signal handler
  signal(SIGCHLD, nullptr);

  // Create a vector of monitor threads that wait for each process to stop.
  std::vector<std::thread> monitors;
  for (const Executable &exec : this->executables)
    monitors.push_back(std::thread([&] {waitpid(exec.pid, nullptr, 0);}));

  // Shutdown the processes
  for (const Executable &exec : this->executables)
  {
    igndbg << "Killing the process[" << exec.name
      << "] with PID[" << exec.pid << "]\n";
    kill(exec.pid, SIGINT);
  }

  igndbg << "Waiting for each process to end\n";

  // Wait for all the monitors to stop
  for (std::thread &m : monitors)
    m.join();
}

//////////////////////////////////////////////////
void ManagerPrivate::LoadPlugin(const std::string &_filename,
    const tinyxml2::XMLElement *_elem)
{
  // Get the plugin's name
  const char *nameStr = _elem->Attribute("name");
  std::string name = nameStr == nullptr ? "" : nameStr;
  if (name.empty())
  {
    ignerr << "Invalid config file[" << _filename << "], "
      << "missing a name attribute for a plugin." << std::endl;
    return;
  }

  // Get the plugin's filename
  const char *fileStr = _elem->Attribute("filename");
  std::string file = fileStr == nullptr ? "" : fileStr;
  if (file.empty())
  {
    ignerr << "Invalid config file[" << _filename << "], "
      << "missing filename attribute for plugin with name[" << name << "]"
      << std::endl;
    return;
  }

  ignition::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv("IGN_LAUNCH_PLUGIN_PATH");
  systemPaths.AddPluginPaths(IGNITION_LAUNCH_PLUGIN_INSTALL_PATH);

  // Add in the gazebo plugin path for convenience
  std::string homePath;
  ignition::common::env(IGN_HOMEDIR, homePath);
  systemPaths.AddPluginPaths(homePath + "/.ignition/gazebo/plugins");

  std::string pathToLib = systemPaths.FindSharedLibrary(file);
  if (pathToLib.empty())
  {
    ignerr << "Failed to find the path to library[" << file << "]. "
      << "Try adding the path to the IGN_LAUNCH_PLUGIN_PATH environment "
      << "variable.\n";
    return;
  }

  plugin::Loader loader;
  std::unordered_set<std::string> localPlugins = loader.LoadLib(pathToLib);
  if (localPlugins.empty())
  {
    ignerr << "Failed to load plugin [camera] : cloud load the "
      << "library\n";
    return;
  }

  std::unordered_set<std::string> validPlugins =
    loader.PluginsImplementing<ignition::launch::Plugin>();
  if (validPlugins.count(name) == 0)
  {
    ignerr << "Failed to find implementation with name[" << name << "] in "
      << file << std::endl;
    return;
  }

  igndbg << "Loading plugin. Name[" << name
    << "] File[" << file << "]" << std::endl;

  PluginPtr plugin = loader.Instantiate(name);
  plugin->QueryInterface<Plugin>()->Load(_elem);
  this->plugins.insert(plugin);
}
