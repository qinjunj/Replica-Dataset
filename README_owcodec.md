### Modified the render script `ReplicaSDK/src/render.cpp` to render datasets with pre-defined camera trajectory. The usage is as in the original [[1]].

### ReplicaRenderer

The ReplicaRenderer shows how to render out images from a Replica for a
programmatically defined trajectory without UI. This executable can be run
headless on a server if so desired. 

```
./build/bin/ReplicaRenderer mesh.ply textures glass.sur
```

[1]: https://github.com/qinjunj/Replica-Dataset/blob/main/README.md