//template<typename T>
__kernel void kalman_predict_2Mat(
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
    int row = get_global_id(1);
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
/*    
    //gentype mad24 (gentype x,
 	//               gentype y,
 	//               gentype z) 
    // ------------------------------------------------------------------------
    // -> Fast integer function to multiply 24-bit integers and add a 32-bit value.
    // mad24 multiplies two 24-bit integer values x and y, 
    // and add the 32-bit integer result to the 32-bit integer z    
    // i.e. mad24(x,y,z) = x * y + z    
    int src1_index = mad24(row, src1_step, col + src1_offset); // #row * step + col = index
    int src2_index = mad24(row, src2_step, col + src2_offset);    
    int dst_index = mad24(row, dst_step, col + dst_offset);
*/
    float sum = 0.0;

    for(size_t i=0; i < (size_t)src1_step; i++)
    {
        sum += src1[row * src1_step + i] * src2[i * src2_step + col];
    }
/*    
    printf("\nsrc1_step: %d", src1_step);    
    printf("\nsrc1_offset: %d", src1_offset);
*/
    dst[row * src2_step + col] = sum;    
};