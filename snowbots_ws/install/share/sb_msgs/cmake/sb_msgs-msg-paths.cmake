# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${sb_msgs_DIR}/.." "msg" sb_msgs_MSG_INCLUDE_DIRS UNIQUE)
set(sb_msgs_MSG_DEPENDENCIES std_msgs)
