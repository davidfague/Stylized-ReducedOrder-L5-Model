: dummy.mod
: Makes a function written in C available to hoc
NEURON {
  SUFFIX nothing
}
: call it gceil to avoid naming conflicts
FUNCTION gceil(z) {
  gceil = ceil(z)
}