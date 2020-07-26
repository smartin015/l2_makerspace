Install:
libgstreamer-plugins-base1.0-dev

To figure out what libraries are listed dependencies:
`ldd demo/bin/x11/libgstreamertexture.so`

Note: if not present, may need to add to SConstruct using `env.Append(LIBS=[...])`

To see what shared library exports:
`nm -D /usr/lib/x86_64-linux-gnu/libgobject-2.0.so.0 | less`
