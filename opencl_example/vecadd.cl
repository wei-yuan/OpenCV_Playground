/* #pragma OPENCL EXTENSION cl_khr_fp64: enable */
__kernel
void vecadd(
            const __global float *a,
            const __global float *b,
            __global float *c)
{
  int id=get_global_id(0);
  c[id] = a[id] + b[id];
  /* c[id] = 123.0; */
}
