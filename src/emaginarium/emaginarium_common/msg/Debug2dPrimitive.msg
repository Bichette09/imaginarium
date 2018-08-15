# this message is used to send 2d primitives to http front-end
# primitive name
string name
# primitive type (point, line, segment)
string type
# primitive color
string color
# primitive def, if empty primitive will be cleared
# - line : [a,b] => y = ax + b
float32[] data


