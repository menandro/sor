#include "CudaFlow.h"

/// image to downscale
texture<float, 2, cudaReadModeElementType> texFine;

__global__ void DownscaleKernel(int width, int height, int stride, float *out)
{
	const int ix = threadIdx.x + blockIdx.x * blockDim.x;
	const int iy = threadIdx.y + blockIdx.y * blockDim.y;

	if (ix >= width || iy >= height)
	{
		return;
	}

	float dx = 1.0f / (float)width;
	float dy = 1.0f / (float)height;

	float x = ((float)ix + 0.5f) * dx;
	float y = ((float)iy + 0.5f) * dy;

	int pos = ix + iy * stride;

	float up = tex2D(texFine, x - dx * 0.25f, y);
	float down = tex2D(texFine, x + dx * 0.25f, y);
	float left = tex2D(texFine, x, y - dy * 0.25f);
	float right = tex2D(texFine, x, y + dy * 0.25f);
	float here = tex2D(texFine, x, y);

	out[pos] = here;
	//out[pos] = 0.2f * (here + up + down + left + right);
	//out[pos] = 0.5f * (here + 0.25f*up + 0.25f*down + 0.25f*left + 0.25f*right);

	/*out[pos] = 0.25f * (tex2D(texFine, x - dx * 0.25f, y) + tex2D(texFine, x + dx * 0.25f, y) +
		tex2D(texFine, x, y - dy * 0.25f) + tex2D(texFine, x, y + dy * 0.25f));*/
}

__global__ void DownscaleNearestNeighborKernel(int width, int height, int stride, float *out)
{
	const int ix = threadIdx.x + blockIdx.x * blockDim.x;
	const int iy = threadIdx.y + blockIdx.y * blockDim.y;

	if (ix >= width || iy >= height)
	{
		return;
	}

	float dx = 1.0f / (float)width;
	float dy = 1.0f / (float)height;

	float x = ((float)ix + 0.5f) * dx;
	float y = ((float)iy + 0.5f) * dy;

	int pos = ix + iy * stride;

	float up = tex2D(texFine, x - dx * 0.25f, y);
	float down = tex2D(texFine, x + dx * 0.25f, y);
	float left = tex2D(texFine, x, y - dy * 0.25f);
	float right = tex2D(texFine, x, y + dy * 0.25f);
	float here = tex2D(texFine, x, y);

	//out[pos] = 0.5f * (here + 0.25f*up + 0.25f*down + 0.25f*left + 0.25f*right);
	//if (up == 1.0f || down == 1.0f || left == 1.0f || right == 1.0f)
	if (here == 0.0f) {
		if (up != 0.0f) out[pos] = up;
		else if (down != 0.0f) out[pos] = down;
		else if (left != 0.0f) out[pos] = left;
		else if (right != 0.0f) out[pos] = right;
		else out[pos] = here;
	}
	else
		out[pos] = here;
}

__global__ void DownscaleMaskKernel(int width, int height, int stride, float *out)
{
	const int ix = threadIdx.x + blockIdx.x * blockDim.x;
	const int iy = threadIdx.y + blockIdx.y * blockDim.y;

	if (ix >= width || iy >= height)
	{
		return;
	}

	float dx = 1.0f / (float)width;
	float dy = 1.0f / (float)height;

	float x = ((float)ix + 0.5f) * dx;
	float y = ((float)iy + 0.5f) * dy;

	int pos = ix + iy * stride;

	float up = tex2D(texFine, x - dx * 0.25f, y);
	float down = tex2D(texFine, x + dx * 0.25f, y);
	float left = tex2D(texFine, x, y - dy * 0.25f);
	float right = tex2D(texFine, x, y + dy * 0.25f);
	float here = tex2D(texFine, x, y);

	//out[pos] = 0.5f * (here + 0.25f*up + 0.25f*down + 0.25f*left + 0.25f*right);
	//if (up == 1.0f || down == 1.0f || left == 1.0f || right == 1.0f)
	if (here == 0.0f) {
		if (up != 0.0f) out[pos] = up;
		else if (down != 0.0f) out[pos] = down;
		else if (left != 0.0f) out[pos] = left;
		else if (right != 0.0f) out[pos] = right;
		else out[pos] = here;
	}
	else
		out[pos] = here;
}

__global__ void DownscaleScalingKernel(int width, int height, int stride, float scale, float *out)
{
	const int ix = threadIdx.x + blockIdx.x * blockDim.x;
	const int iy = threadIdx.y + blockIdx.y * blockDim.y;

	if (ix >= width || iy >= height)
	{
		return;
	}

	float dx = 1.0f / (float)width;
	float dy = 1.0f / (float)height;

	float x = ((float)ix + 0.5f) * dx;
	float y = ((float)iy + 0.5f) * dy;

	int pos = ix + iy * stride;

	//out[pos] = scale * tex2D(texFine, x, y);

	float up = tex2D(texFine, x - dx * 0.25f, y);
	float down = tex2D(texFine, x + dx * 0.25f, y);
	float left = tex2D(texFine, x, y - dy * 0.25f);
	float right = tex2D(texFine, x, y + dy * 0.25f);
	float here = tex2D(texFine, x, y);

	//out[pos] = 0.5f * (here + 0.25f*up + 0.25f*down + 0.25f*left + 0.25f*right);
	//if (up == 1.0f || down == 1.0f || left == 1.0f || right == 1.0f)
	if (here == 0.0f) {
		if (up != 0.0f) out[pos] = scale * up;
		else if (down != 0.0f) out[pos] = scale * down;
		else if (left != 0.0f) out[pos] = scale * left;
		else if (right != 0.0f) out[pos] = scale * right;
		else out[pos] = scale * here;
	}
	else
		out[pos] = scale * here;
}

void sor::CudaFlow::Downscale(const float *src, int width, int height, int stride,
	int newWidth, int newHeight, int newStride, float *out)
{
	dim3 threads(BlockWidth, BlockHeight);
	dim3 blocks(iDivUp(newWidth, threads.x), iDivUp(newHeight, threads.y));

	// mirror if a coordinate value is out-of-range
	texFine.addressMode[0] = cudaAddressModeMirror;
	texFine.addressMode[1] = cudaAddressModeMirror;
	texFine.filterMode = cudaFilterModeLinear;
	texFine.normalized = true;

	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();

	size_t offset;
	checkCudaErrors(cudaBindTexture2D(&offset, texFine, src, desc, width, height, stride * sizeof(float)));

	DownscaleKernel <<< blocks, threads >>>(newWidth, newHeight, newStride, out);
}

void sor::CudaFlow::DownscaleMask(const float *src, int width, int height, int stride,
	int newWidth, int newHeight, int newStride, float *out)
{
	dim3 threads(BlockWidth, BlockHeight);
	dim3 blocks(iDivUp(newWidth, threads.x), iDivUp(newHeight, threads.y));

	// mirror if a coordinate value is out-of-range
	texFine.addressMode[0] = cudaAddressModeMirror;
	texFine.addressMode[1] = cudaAddressModeMirror;
	texFine.filterMode = cudaFilterModePoint;
	texFine.normalized = true;

	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();

	size_t offset;
	checkCudaErrors(cudaBindTexture2D(&offset, texFine, src, desc, width, height, stride * sizeof(float)));

	DownscaleMaskKernel << < blocks, threads >> >(newWidth, newHeight, newStride, out);
}

void sor::CudaFlow::DownscaleNearestNeighbor(const float *src, int width, int height, int stride,
	int newWidth, int newHeight, int newStride, float *out)
{
	dim3 threads(BlockWidth, BlockHeight);
	dim3 blocks(iDivUp(newWidth, threads.x), iDivUp(newHeight, threads.y));

	// mirror if a coordinate value is out-of-range
	texFine.addressMode[0] = cudaAddressModeMirror;
	texFine.addressMode[1] = cudaAddressModeMirror;
	texFine.filterMode = cudaFilterModeLinear;
	texFine.normalized = true;

	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();

	size_t offset;
	checkCudaErrors(cudaBindTexture2D(&offset, texFine, src, desc, width, height, stride * sizeof(float)));

	DownscaleNearestNeighborKernel << < blocks, threads >> >(newWidth, newHeight, newStride, out);
}

void sor::CudaFlow::Downscale(const float *src, int width, int height, int stride,
	int newWidth, int newHeight, int newStride, float scale, float *out)
{
	dim3 threads(BlockWidth, BlockHeight);
	dim3 blocks(iDivUp(newWidth, threads.x), iDivUp(newHeight, threads.y));

	// mirror if a coordinate value is out-of-range
	texFine.addressMode[0] = cudaAddressModeMirror;
	texFine.addressMode[1] = cudaAddressModeMirror;
	texFine.filterMode = cudaFilterModePoint;
	texFine.normalized = true;

	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();

	size_t offset;
	checkCudaErrors(cudaBindTexture2D(&offset, texFine, src, desc, width, height, stride * sizeof(float)));

	DownscaleScalingKernel << < blocks, threads >> >(newWidth, newHeight, newStride, scale, out);
}