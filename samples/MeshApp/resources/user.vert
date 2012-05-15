// Inputs
uniform vec3 scale;
uniform sampler2D positions;

// Properties
varying float brightness;
varying vec3 normal;
varying vec4 position;
varying vec4 uv;

// Kernel
void main( void )
{

	// Get texture coordinate
	uv = gl_MultiTexCoord0;

	// Get brightness from first channel (assumes image is greyscale)
	brightness = texture2D( positions, uv.st ).r;

	// Get normal
	normal = normalize( gl_NormalMatrix * gl_Normal );

	// Get position in VBO
	position = gl_Vertex;

	// Scale position
	position.x = -position.x * scale.x;
	position.y = position.y * scale.y;
	position.z = brightness * scale.z;

	// Transform position
	gl_Position = gl_ModelViewProjectionMatrix * position;

}
