string PG_COLS="all project_id>Project"

int64 id
int64 project_id

# https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/Location
# A URL pointing to where the content was fetched from.
# e.g. l2_storage://tasks/123456789
string location

# The original / public location where the item came from.
# If this is empty, then the origin is the location.
# e.g. https://todoist.com/app/#task/123456789
string origin 

# A media type. In addition to
# regular file formats, this could
# also be any of the constants defined after.
# 
# https://developer.mozilla.org/en-US/docs/Glossary/MIME_type
string content_type
string CONTENT_TYPE_L2_TASK="text/l2task"

# Only one of these should be populated.
# We include a string in addition to byte arrays because
# it's very common to be passing strings around.
# It's easier to read messages via `ros2 topic echo`
# and there's less casting when treated specially here.
string content
byte[] data
