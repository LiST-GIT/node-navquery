const fs = require( 'fs' );
const recast = require( '..' );

let sample = new recast.NavQuery();
let result = null;
for ( let index = 0; index < 1; index++ ) {
	sample = new recast.NavQuery();
	result = sample.load( __dirname + '/tutorial.bin' );
}

if ( sample.load( __dirname + '/tutorial.bin' ) ) {
	let start = sample.findNearestPoly( 76, 0, 111, 2, 1000, 2 );
	let end = sample.findNearestPoly( 80, 0, 120, 2, 1000, 2 );
	end = sample.findRandomPoint();
	for ( let index = 0; index < 10; index++ ) {
		let p1 = sample.findRandomPoint();
		console.time( 'findStraightPath' );
		let result = sample.findStraightPath( p1, end );
		console.timeEnd( 'findStraightPath' );
		console.log( typeof result !== 'object' ? result : JSON.stringify( result.map( data => [ ~~data.x, ~~data.z ] ) ) );
	}
}
