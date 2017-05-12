attribute vec2 a_vertex;
attribute vec2 a_uv;
varying vec2 v_uv;

void main(void)
{
	gl_Position = vec4(a_vertex, 0.0, 1.0);
	v_uv = a_uv;
}