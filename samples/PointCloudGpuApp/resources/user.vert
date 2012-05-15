// Inputs
uniform float depth;
uniform vec3 scale;
uniform sampler2D tex;

// Properties
varying float brightness;
varying vec3 color;
varying vec4 vertex;

// Kernel
void main( void )
{

	// Get brightness from first channel (assumes image is greyscale)
	brightness = texture2D( tex, gl_MultiTexCoord0.st ).r;

	// Get position
	vertex = vec4( gl_Vertex );

	// Scale position
	vertex.x = -vertex.x * scale.x;
	vertex.y = vertex.y * scale.y;
	vertex.z = depth * brightness * scale.z;

	// Transform position
	gl_Position = gl_ModelViewProjectionMatrix * vertex;

}
