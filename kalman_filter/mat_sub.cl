__kernel void mat_add(
   int mat_type,
   __global float* src1,
   int src1_step, int src1_offset,
   __global float* src2,
   int src2_step, int src2_offset,
   __global float* dst,
   int dst_step, int dst_offset,
   int dst_rows, int dst_cols)
{    
    int col = get_global_id(0);
    int row = get_global_id(1); // no need for thiss
    //end condition
    if (col >= dst_cols) return;

    // spec conversion
    if(mat_type == 5) // CV_32FC1: 5
    {    
        src1_step /= 4;
        src2_step /= 4;
        src1_offset /= 4;
        src2_offset /= 4;
    }
    
    int index = row * src1_step + (col + src1_offset);
    
    dst[index] = src1[index] - src2[index];    
};