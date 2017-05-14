
#ifdef GL_ES
precision mediump float;
#endif
uniform sampler2D inputTexture;
varying vec2 v_uv;

vec3 colorConv(vec3 rgb)
{
	const mat3 rgb2xyz = mat3(
		0.412453, 0.357590, 0.180423,
		0.212671, 0.715160, 0.072169,
		0.019334, 0.119193, 0.950227);
	const mat3 b = mat3(
		9.465229e-1, 2.946927e-1, -1.313419e-1,
		-1.179179e-1, 9.929960e-1, 7.371554e-3,
		9.230461e-2, -4.645794e-2, 9.946464e-1);
	const mat3 a = mat3(
		2.707439e1, -2.280783e1, -1.806681,
		-5.646736, -7.722125, 1.286503e1,
		-4.163133, -4.579428, -4.576049);
	return (a * log(b * rgb2xyz * rgb));
}


// Taken from lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
vec3 rgb2hsv(vec3 c)
{
	vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
	vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
	vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

	float d = q.x - min(q.w, q.y);
	float e = 1.0e-10;
	return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

// Converts a single linear channel to srgb
float linear_to_srgb(float channel) {
	if (channel <= 0.0031308)
		return 12.92 * channel;
	else
		return (1.0 + 0.055) * pow(channel, 1.0 / 2.4) - 0.055;
}

vec3 rgb2xyz(vec3 rgb) {
	const mat3 rgb2xyz = mat3(
		0.412453, 0.357590, 0.180423,
		0.212671, 0.715160, 0.072169,
		0.019334, 0.119193, 0.950227);
	

	//rgb.r = linear_to_srgb(rgb.r);
	//rgb.g = linear_to_srgb(rgb.g);
	//rgb.b = linear_to_srgb(rgb.b);

	/*const mat3 rgb2xyz = mat3(
		0.412456439089692, 0.212672851405623, 0.019333895582329,
		0.357576077643909, 0.715152155287818, 0.119192025881303,
		0.180437483266399, 0.072174993306560, 0.950304078536368);
*/
	vec3 xyz = rgb * rgb2xyz;
	return xyz;
}

float lab_f(float t) {
	if(t > 0.008856)
		return pow(t, 1.0 / 3.0);
	return (t * 903.3 + 16.0) / 116.0;
}

vec3 xyz2lab(vec3 xyz) {
	float x = xyz.x/0.950456;
	float y = xyz.y;
	float z = xyz.z/1.088754;
	float fX = x > 0.008856 ? pow(x, 1.0 / 3.0) : (x * 903.3 + 16.0) / 116.0;
	float fY = y > 0.008856 ? pow(y, 1.0 / 3.0) : (y * 903.3 + 16.0) / 116.0;
	float fZ = z > 0.008856 ? pow(z, 1.0 / 3.0) : (z * 903.3 + 16.0) / 116.0;
	float L = y > 0.008856 ? 116.0 * pow(y, 1.0/3.0) - 16.0 : 903.3 * y;
	float a = 500.0 * (fX - fY);
	float b = 200.0 * (fY - fZ);
	return vec3(L / 100.0, (a + 128.0) / 255.0, (b + 128.0) / 255.0);
}

vec3 rgb2lab(in vec3 rgb){
    vec3 xyz = rgb2xyz(rgb);
    return xyz2lab(xyz);
}

void main(void)
{
	vec3 rgb = texture2D(inputTexture, v_uv).bgr;

	//vec3 cspace = colorConv(rgb);
	//vec3 cspace = rgb2hsv(rgb);
	//vec3 cspace = rgb2xyz(rgb);
	vec3 cspace = rgb2lab(rgb);

	gl_FragData[0] = vec4(cspace, 1.0);
}