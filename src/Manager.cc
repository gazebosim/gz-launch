/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <fcntl.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <tinyxml2.h>
#include <unistd.h>

#include <condition_variable>
#include <list>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/plugin/Loader.hh>

#include "ignition/launch/config.hh"
#include "ignition/launch/Plugin.hh"
#include "Manager.hh"

using namespace ignition::launch;
using namespace std::chrono_literals;

static constexpr const char* kSemaphoreName = "/child_semaphore";

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
  /// \param[in] _envs Environment variables to set.
  public: Executable(const std::string &_name, const pid_t _pid,
            const std::vector<std::string> &_cmd, bool _autoRestart,
            const std::list<std::string> &_envs)
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
  public: std::list<std::string> envs;
};

/// \brief Private data variables for the Ignition class.
class ignition::launch::ManagerPrivate
{
  /// \brief Constructor.
  public: ManagerPrivate();

  /// \brief Destructor.
  public: ~ManagerPrivate();

  /// \brief Parse a configuration string.
  /// \param[in] _string XML configuration string.
  /// \return True on success.
  public: bool ParseConfig(const std::string &_string);

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
  /// \param[in] _envs Environment variables to set.
  /// \return True on success.
  public: bool RunExecutable(const std::string &_name,
    const std::vector<std::string> &_cmd,
    const bool _autoRestart,
    const std::list<std::string> &_envs);

  /// \brief Stop all executables
  public: void ShutdownExecutables();

  /// \brief Stop running. This can be used to end a Run().
  /// \return True if running, False if the state was not running.
  public: bool Stop();

  /// \brief Load a plugin based on data contained in an XML element.
  /// \param[in] _elem Pointer to the XML element containing the plugin
  /// information.
  public: void LoadPlugin(const tinyxml2::XMLElement *_elem);

  /// \brief Handle SIG_INT and SIG_TERM signals
  /// \param[in] _sig The signal
  private: void OnSigIntTerm(int _sig);

  /// \brief Handle SIG_CHILD
  /// \param[in] _sig The signal
  private: static void OnSigChild(int _sig);

  /// \brief Start the worker thread to service stopped children.
  public: void StartWorkerThread();

  /// \brief Thread to handle restarting stopped children.
  private: void RestartLoop();

  /// \brief Parse executable configurations.
  /// \param[in] _elem XML element that contains an <executable>
  private: void ParseExecutables(const tinyxml2::XMLElement *_elem);

  /// \brief Parse <env> elements.
  /// \return List of environment variable name,value pairs.
  private: std::list<std::string> ParseEnvs(const tinyxml2::XMLElement *_elem);

  /// \brief Set environment variables.
  /// \param[in] _envs List of environment variable name,value pairs.
  private: void SetEnvs(const std::list<std::string> &_envs);

  /// \brief Parse executable wrappers. Executable wrappers allow a plugin
  /// to run in a process.
  /// \param[in] _elem  XML element that contains an <executable_wrapper>
  private: void ParseExecutableWrappers(const tinyxml2::XMLElement *_elem);

  /// \brief A list of executables that are running, or have been run.
  public: std::list<Executable> executables;

  /// \brief A list of children that were stopped to attempt restarts
  public: std::queue<pid_t> stoppedChildren;

  /// \brief Semaphore to prevent restartThread from being a spinlock
  private: sem_t *stoppedChildSem;

  /// \brief Thread containing the restart loop
  private: std::thread restartThread;

  /// \brief All the plugins
  public: std::unordered_set<launch::PluginPtr> plugins;

  /// \brief All the wrapped plugins
  public: std::list<pid_t> wrappedPlugins;

  /// \brief Mutex to protect the executables list.
  public: std::mutex executablesMutex;

  /// \brief A condition variable used by SIG_INT and Manager::Run.
  public: std::condition_variable runCondition;

  /// \brief A mutex used in conjunction with runCondition.
  public: std::mutex runMutex;

  /// \brief True if running.
  public: std::atomic<bool> running = false;

  /// \brief True indicates that this process is the master (main) process.
  public: bool master = false;

  /// \brief Our signal handler.
  public: std::unique_ptr<common::SignalHandler> sigHandler = nullptr;

  /// \brief Top level environment variables.
  public: std::list<std::string> envs;

  /// \brief Pointer to myself. This is used in the signal handlers.
  /// A raw pointer is acceptable here since it is used only internally.
  public: static ManagerPrivate *myself;
};

// Init the static pointer.
ManagerPrivate *ManagerPrivate::myself = nullptr;

/////////////////////////////////////////////////
Manager::Manager()
  :dataPtr(new ManagerPrivate())
{
  this->dataPtr->myself = this->dataPtr.get();

  std::string homePath;
  ignition::common::env(IGN_HOMEDIR, homePath);

  // Make sure to initialize logging.
  ignLogInit(homePath + "/.ignition", "launch.log");
  if (!this->dataPtr->sigHandler->Initialized())
    ignerr << "signal(2) failed while setting up for SIGINT" << std::endl;
}

/////////////////////////////////////////////////
Manager::~Manager()
{
  this->dataPtr->Stop();
}

/////////////////////////////////////////////////
bool Manager::RunConfig(const std::string &_config)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->runMutex);

  // This is the master.
  this->dataPtr->master = true;

  // Parse the configuration string, and run all the specified commands.
  if (!this->dataPtr->ParseConfig(_config))
    return false;

  // Child processes should exit
  if (!this->dataPtr->master)
    return true;

  // Get whether or not we should run (block).
  this->dataPtr->running = !this->dataPtr->executables.empty() ||
                           !this->dataPtr->plugins.empty();

  // Start thread to service child signals.
  this->dataPtr->StartWorkerThread();

  // Wait for a shutdown event, or for all the executables to quit.
  while (this->dataPtr->running && (!this->dataPtr->executables.empty() ||
                                    !this->dataPtr->plugins.empty()))
  {
    this->dataPtr->runCondition.wait(lock);
  }
  this->dataPtr->running = false;

  // Stop plugins.
  this->dataPtr->plugins.clear();

  // Stop executables.
  this->dataPtr->ShutdownExecutables();

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
  // Register a normal signal handler
  this->sigHandler.reset(new common::SignalHandler());
  this->sigHandler->AddCallback(
      std::bind(&ManagerPrivate::OnSigIntTerm, this, std::placeholders::_1));

  // Initialize semaphore
  this->stoppedChildSem = sem_open(kSemaphoreName, O_CREAT, 0644, 1);
  if (this->stoppedChildSem == SEM_FAILED) {
    ignerr << "Error initializing semaphore: " << strerror(errno) << std::endl;
  }

  // Register a signal handler to capture child process death events.
  if (signal(SIGCHLD, ManagerPrivate::OnSigChild) == SIG_ERR)
    ignerr << "signal(2) failed while setting up for SIGCHLD" << std::endl;
}

/////////////////////////////////////////////////
ManagerPrivate::~ManagerPrivate()
{
  if (sem_close(this->stoppedChildSem) == -1)
  {
    ignerr << "Failed to close semaphore: " << strerror(errno) << std::endl;
  }

  if (sem_unlink(kSemaphoreName) == -1)
  {
    ignerr << "Failed to unlink semaphore: " << strerror(errno) << std::endl;
  }
}

/////////////////////////////////////////////////
bool ManagerPrivate::Stop()
{
  if (this->runMutex.try_lock())
  {
    if (this->running)
    {
      this->running = false;
    }
    this->runMutex.unlock();
    this->runCondition.notify_all();

    // Signal the restart thread to stop
    sem_post(this->stoppedChildSem);
    if (this->restartThread.joinable())
      this->restartThread.join();
  }
  return this->running;
}

/////////////////////////////////////////////////
void ManagerPrivate::OnSigIntTerm(int _sig)
{
  igndbg << "OnSigIntTerm Received signal[" << _sig  << "]\n";
  this->Stop();
}

/////////////////////////////////////////////////
void ManagerPrivate::OnSigChild(int _sig)
{
  // This is a signal handler, so be careful not to do any operations
  // that you are not allowed to do.
  // Ref: http://man7.org/linux/man-pages/man7/signal-safety.7.html
  (void) _sig;  // Commenting _sig above confuses codecheck
  pid_t p;
  int status;

  // Retreive the stopped child's PID, append, and signal the consumer.
  if ((p = waitpid(-1, &status, WNOHANG)) != -1)
  {
    myself->stoppedChildren.push(p);
    sem_post(myself->stoppedChildSem);
  }
}

/////////////////////////////////////////////////
void ManagerPrivate::StartWorkerThread()
{
  // Spawn thread to monitor restarting children.
  this->restartThread = std::thread(&ManagerPrivate::RestartLoop, this);
}

/////////////////////////////////////////////////
void ManagerPrivate::RestartLoop()
{
  sigset_t chldmask;

  if ((sigemptyset(&chldmask) == -1) || (sigaddset(&chldmask, SIGCHLD) == -1))
  {
    ignerr << "Failed to initialize signal mask: "
      << strerror(errno) << std::endl;
    return;
  }

  while (this->running)
  {
    // Wait for the signal handler to signal that a child has exited.
    int s = sem_wait(this->stoppedChildSem);
    if (s == -1)
    {
      if (errno != EINTR)
        ignwarn << "sem_wait error: "
          << strerror(errno) << std::endl;
      continue;
    }

    // Block SIGCHLD while consuming queue.
    if (sigprocmask(SIG_BLOCK, &chldmask, NULL) == -1)
    {
      ignerr << "Failed to setup block for SIGCHLD: "
        << strerror(errno) << std::endl;
      continue;
    }

    // Consume stopped children from the queue.
    while (!this->stoppedChildren.empty())
    {
      std::lock_guard<std::mutex> mutex(this->executablesMutex);

      // Executable to restart
      Executable restartExec;

      pid_t p = this->stoppedChildren.front();
      this->stoppedChildren.pop();

      // Find the executable
      for (std::list<Executable>::iterator iter = this->executables.begin();
           iter != this->executables.end(); ++iter)
      {
        if (iter->pid == p)
        {
          igndbg << "Death of process[" << p << "] with name["
                 << iter->name << "].\n";

          // Restart if autoRestart is enabled
          if (iter->autoRestart)
            restartExec = *iter;

          this->executables.erase(iter);
          break;
        }
      }

      if (!restartExec.name.empty() && !restartExec.command.empty())
      {
        igndbg << "Restarting process with name[" << restartExec.name << "]\n";
        this->RunExecutable(restartExec);
      }

      this->runCondition.notify_all();
    }

    // Unblock SIGCHLD
    if (sigprocmask(SIG_UNBLOCK, &chldmask, NULL) == -1)
    {
      ignerr << "Failed to unblock SIGCHLD: " << strerror(errno) << std::endl;
      continue;
    }
  }
}


/////////////////////////////////////////////////
bool ManagerPrivate::ParseConfig(const std::string &_config)
{
  tinyxml2::XMLDocument xmlDoc;

  // Load the XML configuration file into TinyXML
  if (xmlDoc.Parse(_config.c_str()) != tinyxml2::XML_SUCCESS)
  {
    ignerr << "Unable to parse configuration. Your XML might be invalid.\n";
    return false;
  }

  // Get the root element.
  tinyxml2::XMLElement *root = xmlDoc.FirstChildElement("ignition");
  if (!root)
  {
    ignerr << "Invalid config file,m issing <ignition> element\n";
    return false;
  }

  // Parse and create all the <executable> elements.
  this->ParseExecutables(root);

  // Parse and create all the <executable_wrapper> elements.
  if (this->master)
    this->ParseExecutableWrappers(root);

  // Keep the environment variables in memory. See manpage for putenv.
  this->envs = this->ParseEnvs(root);
  this->SetEnvs(this->envs);

  // Parse and create all the <plugin> elements.
  if (this->master)
  {
    // Process all the plugins.
    tinyxml2::XMLElement *pluginElem = root->FirstChildElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->NextSiblingElement("plugin");
    }
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
    const std::list<std::string> &_envs)
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
    this->master = true;
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

    this->SetEnvs(_envs);

    // Run the command, replacing the current process image
    if (execvp(cstrings[0], &cstrings[0]) < 0)
    {
      ignerr << "Unable to run command["
        << std::accumulate(_cmd.begin(), _cmd.end(), std::string("")) << "]\n";
      return false;
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

  for (const pid_t &wrapper : this->wrappedPlugins)
    monitors.push_back(std::thread([&] {waitpid(wrapper, nullptr, 0);}));

  // Shutdown the processes
  for (const Executable &exec : this->executables)
  {
    igndbg << "Killing the process[" << exec.name
      << "] with PID[" << exec.pid << "]\n";
    kill(exec.pid, SIGINT);
  }

  // Shutdown the wrapped plugins
  for (const pid_t &pid : this->wrappedPlugins)
  {
    igndbg << "Killing the wrapped plugin PID[" << pid << "]\n";
    kill(pid, SIGINT);
  }

  igndbg << "Waiting for each process to end\n";

  // Wait for all the monitors to stop
  for (std::thread &m : monitors)
    m.join();
}

//////////////////////////////////////////////////
void ManagerPrivate::ParseExecutables(const tinyxml2::XMLElement *_elem)
{
  // Process all the executables.
  const tinyxml2::XMLElement *execElem = _elem->FirstChildElement("executable");

  // This "i" variable is just used for output messages.
  for (int i = 0; execElem && this->master; ++i)
  {
    bool autoRestart = false;
    bool valid = true;
    std::vector<std::string> cmdParts;

    // Get the executable's name
    std::string nameStr = execElem->Attribute("name");
    if (nameStr.empty())
    {
      valid = false;
      ignerr << "Invalid configuration file, "
        << "missing name attribute for the " << i << " <executable> element."
        << std::endl;
    }

    // Get the command
    const tinyxml2::XMLElement *cmdElem =
      execElem->FirstChildElement("command");
    if (!cmdElem)
    {
      valid = false;
      ignerr << "Invalid configuration file, "
        << "missing <command> child element "
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
    const tinyxml2::XMLElement *restartElem = execElem->FirstChildElement(
        "auto_restart");
    if (restartElem && restartElem->GetText())
    {
      std::string txt = ignition::common::lowercase(restartElem->GetText());
      autoRestart = txt == "true" || txt == "1" || txt == "t";
    }

    std::list<std::string> localEnvs = this->ParseEnvs(execElem);

    if (valid)
    {
      if (!this->RunExecutable(nameStr, cmdParts, autoRestart, localEnvs))
      {
        ignerr << "Unable to run executable named[" << nameStr << "] in "
          << "configuration file.\n";
      }
    }

    execElem = execElem->NextSiblingElement("executable");
  }
}

//////////////////////////////////////////////////
std::list<std::string> ManagerPrivate::ParseEnvs(
    const tinyxml2::XMLElement *_elem)
{
  std::list<std::string> result;

  // Get the environment variables
  const tinyxml2::XMLElement *envElem = _elem->FirstChildElement("env");
  while (envElem)
  {
    const tinyxml2::XMLElement *nameElem = envElem->FirstChildElement("name");
    const tinyxml2::XMLElement *valueElem =
      envElem->FirstChildElement("value");
    if (nameElem && valueElem)
    {
      std::string name = nameElem->GetText();
      std::string value = valueElem->GetText();

      // Expand env var contents, such as $LD_LIBRARY_PATH
      if (!value.empty() && value.at(0) == '$')
      {
        ignition::common::env(value.substr(1), value);
      }

      result.push_back(name + "=" + value);
    }

    envElem = envElem->NextSiblingElement("env");
  }

  return result;
}

//////////////////////////////////////////////////
void ManagerPrivate::LoadPlugin(const tinyxml2::XMLElement *_elem)
{
  // Get the plugin's name
  const char *nameStr = _elem->Attribute("name");
  std::string name = nameStr == nullptr ? "" : nameStr;
  if (name.empty())
  {
    ignerr << "Invalid config file, "
      << "missing a name attribute for a plugin." << std::endl;
    return;
  }

  // Get the plugin's filename
  const char *fileStr = _elem->Attribute("filename");
  std::string file = fileStr == nullptr ? "" : fileStr;
  if (file.empty())
  {
    ignerr << "Invalid config file, "
      << "missing filename attribute for plugin with name[" << name << "]"
      << std::endl;
    return;
  }

  ignition::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv("IGN_LAUNCH_PLUGIN_PATH");
  systemPaths.AddPluginPaths(IGNITION_LAUNCH_PLUGIN_INSTALL_PATH);

  // Add LD_LIBRARY_PATH
#ifdef __linux__
  std::string libPath;
  ignition::common::env("LD_LIBRARY_PATH", libPath);
  systemPaths.AddPluginPaths(libPath);
#endif

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
    ignerr << "Failed to load plugin [" << pathToLib << "] library\n";
    return;
  }

  std::unordered_set<std::string> validPlugins =
    loader.PluginsImplementing<ignition::launch::Plugin>();
  if (validPlugins.count(name) == 0)
  {
    std::string availablePlugins;
    for (const std::string &vp : validPlugins)
      availablePlugins += vp + " ";

    ignerr << "Failed to find implementation with name[" << name << "] in "
      << file << ". Available implementations include: "
      << availablePlugins << std::endl;
    return;
  }

  igndbg << "Loading plugin. Name[" << name
    << "] File[" << file << "]" << std::endl;

  PluginPtr plugin = loader.Instantiate(name);
  if (plugin->QueryInterface<Plugin>()->Load(_elem))
    this->plugins.insert(plugin);
}

//////////////////////////////////////////////////
void ManagerPrivate::ParseExecutableWrappers(
  const tinyxml2::XMLElement *_elem)
{
  // Process all the executables.
  const tinyxml2::XMLElement *execElem = _elem->FirstChildElement(
      "executable_wrapper");
  std::list<pid_t> pluginPids;

  // This "i" variable is just used for output messages.
  for (int i = 0; execElem && this->master; ++i)
  {
    const tinyxml2::XMLElement *pluginElem =
      execElem->FirstChildElement("plugin");
    if (pluginElem)
    {
      // Fork a process for the command
      pid_t pid = fork();
      // If parent process...
      if (pid)
      {
        this->master = true;
        pluginPids.push_back(pid);
      }
      else
      {
        this->master = false;

        // Remove from foreground process group.
        setpgid(0, 0);

        this->plugins.clear();
        this->wrappedPlugins.clear();
        this->sigHandler.reset();
        this->executables.clear();
        this->LoadPlugin(pluginElem);
        return;
      }
    }
    execElem = execElem->NextSiblingElement("executable_wrapper");
  }

  if (this->master)
    this->wrappedPlugins = pluginPids;
}

//////////////////////////////////////////////////
void ManagerPrivate::SetEnvs(const std::list<std::string> &_envs)
{
  for (const std::string &env : _envs)
    putenv(const_cast<char*>(env.c_str()));
}
