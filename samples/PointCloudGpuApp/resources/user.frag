// Brightness
varying float brightness;

// Kernel
void main( void )
{

	// Set color
	gl_FragColor = vec4( 1.0, 1.0, 1.0, brightness );

}
