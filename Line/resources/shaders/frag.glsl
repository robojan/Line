
precision mediump float;
uniform sampler2D inputTexture;
uniform vec3 threshold1_low;
uniform vec3 threshold2_low;
uniform vec3 threshold3_low;
uniform vec3 threshold1_high;
uniform vec3 threshold2_high;
uniform vec3 threshold3_high;
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

void main(void)
{
	vec3 rgb = texture2D(inputTexture, v_uv).bgr;

	//vec3 cspace = colorConv(rgb);
	vec3 cspace = rgb2hsv(rgb);

	vec3 res;
	res.r = (all(greaterThanEqual(cspace, threshold1_low)) && all(lessThanEqual(cspace, threshold1_high))) ? 1.0 : 0.0;
	res.g = (all(greaterThanEqual(cspace, threshold2_low)) && all(lessThanEqual(cspace, threshold2_high))) ? 1.0 : 0.0;
	res.b = (all(greaterThanEqual(cspace, threshold3_low)) && all(lessThanEqual(cspace, threshold3_high))) ? 1.0 : 0.0;

	gl_FragData[0] = vec4(res, 1.0);
}