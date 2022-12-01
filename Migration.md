# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Launch 2.2.2

- Environment variable `IGN_LAUNCH_CONFIG_PATH` started to be treated as a path
  list (colon-separated on Linux, semicolon-separated on Windows). Before, only
  a single path could be set here, and setting a path list would break the whole
  launch file lookup functionality.

## Ignition Launch 0.X to N.M
