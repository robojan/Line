#ifdef GL_ES
precision mediump float;
#endif
uniform sampler2D inputTexture;
uniform vec3 lowBlue;
uniform vec3 highBlue;
uniform vec3 lowRed;
uniform vec3 highRed;
uniform vec3 lowYellow;
uniform vec3 highYellow;
uniform float horizon;
varying vec2 v_uv;


void main(void)
{
	// TODO go back to using the horizon
	vec2 uv = vec2(v_uv.x, v_uv.y / horizon);
	vec3 lab = texture2D(inputTexture, v_uv).rgb;
	
	vec3 res;
	res.r = (all(greaterThanEqual(lab, lowBlue)) && all(lessThanEqual(lab, highBlue))) ? 1.0 : 0.0;
	res.g = (all(greaterThanEqual(lab, lowRed)) && all(lessThanEqual(lab, highRed))) ? 1.0 : 0.0;
	res.b = (all(greaterThanEqual(lab, lowYellow)) && all(lessThanEqual(lab, highYellow))) ? 1.0 : 0.0;
	
	gl_FragData[0] = vec4(res, 1.0);
}