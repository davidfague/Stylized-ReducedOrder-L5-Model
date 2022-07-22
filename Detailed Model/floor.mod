: dummy.mod
: Makes a function written in C available to hoc
NEURON {
  SUFFIX nothing
}
: call it gfloor to avoid naming conflicts
FUNCTION gfloor(z) {
  gfloor = floor(z)
}