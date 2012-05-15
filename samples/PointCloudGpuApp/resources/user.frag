// Brightness
varying float brightness;

// Kernel
void main( void )
{

	// Set color
	if ( brightness < 0.8 ) {
		gl_FragColor = vec4( 1.0, 1.0, 1.0, brightness );
	} else {
		gl_FragColor = vec4( 0.0, 0.0, 0.0, 0.0 );
	}

}
