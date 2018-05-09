Ergodic Manual

How to use Ergodicity in Euclidean motion group E 2

Conan Veitch

M.Sc Candidate, Computer Science

University of Northern British Columbia

Prince George, BC

veitch@unbc.ca


What Are We Doing?

We are implementing the metric of ergodicity and ergodic dynamics developed by Mathew and Meziĉ
in a multi robot system. In order to do so, we leverage Miller et all’s work on projection-based gradient
descent in order to optimize the ergodic trajectory of each robot. Much of this work is based on the
Julia implementation of Miller’s work provided by Louis Dressel.
The intent of this work is to allow a comparison between the trajectory through a domain of a
robot to be compared to an ideal ergodic trajectory, then use this knowledge to make a control decision
(which state to enter next). This is accomplished via a metric that compares the Fourier coefficients
of our robots trajectory to the Fourier coefficients of the ideal trajectory over our domain. This result
is then optimized via gradient descent.


How Do We Accomplish This?

There are a series of steps we must take in order to have a robot follow an ergodic search trajectory:

• Find a domain D. We must have a rectangular domain in R n . We will use the group E 2 (described
below) as our domain, as it fulfils all requirements.

• Find a distribution φ over D. We must have a distribution over our domain that we call the
spatial distribution. This distribution represents information density over D.

• A fourier basis function F k (x) for D. Since E 2 is our domain, we can use the basis function that
Miller and Murphy have provided. This basis function is in terms of the Bessel function, which
is itself sinusoidal - allowing for it to act as a Fourier basis function.

• We must then decompose φ into Fourier coefficients - possible since we have described φ in terms
of the Bessel function.

• The trajectory of the robot must then be described as distribution c over the domain, then
decomposed into Fourier coefficients. c is described in terms of the Dirac delta function.

• We then call ε the sum of squared differences of c and φ, weighted by a factor Λ. ε is our metric
of ergodicity.


What is E 2 ?

E 2 is the special Euclidean motion group in two dimensions. Each element in this group can be thought
of as a state in time: an x coordinate, a y coordinate, and an angle θ (this angle is measured from π,
counter clockwise). It is the group of isometries of Euclidean space in two dimensions (R 2 ). That is,
it’s elements are linear transformations that preserve length - rotation and translation. The operation
on this group is function composition (eg: translation, and rotation).


Why Bother With E 2 ?

The reason we have chosen this particular domain for our ergodic search domain is because we are
implementing a metric of ergodicity physically - in a multi robot system. Given that our robots have
positions and bearings, E 2 then supplies us with a domain that caters to our multi robot systems’
construction. An infrared detector is positioned in the front of each robot, and as such the detector
expects information to be supplied in that position.


Steps for Ergodicity in E 2

• Create domain D. The D is a structure that contains all of the states of E 2 that we are interested
in. In this way we restrict D’s boundaries to some rectangle in R 2 .

– Create a domain data structure that contains information about D.

– Domain structure contains the bottom left and top right boundary coordinates the rectan-
gular space that makes up D.

– We are using E 2 as D, so we also need to add the angles −π to π in order to cover the entire
angular space.

– D can then be described as a vector [r, ψ, θ] T using polar coordinates.

– Discretize D. Split the domain into discrete sub-rectangles. Given a domain that is p meters
X q meters, we divide that into n smaller, but identical squares. Eg: You could split a 1m
x 1m square into 100 1cm x 1cm chunks.

• Create a probability distribution φ over this domain - φ denotes information density over the
domain. Coordinates that we would like our trajectory to spend more time within will be weighted
heavier than coordinates we do not want to spend time within.

– Represent φ as an array with n dimensions (n = 2 for E 2 ). The length of each dimension
of the array is equal to the number of sub-rectangles we discretized the domain into.

– φ is created via functions with the domain as an argument. We require a function in order
to populate the array and one to normalize the data. Initially we require only that a single
circular area of high information density exists, but we will need to implement a way to
create gaussians eventually in order to create arbitrary PDFs (arbitrary, but still normal).

• Ergodic Construction. An ergodic controller must be created that takes D, φ, and the number
of Fourier coefficients of φ. We will need a way to update φ, breaking it down into its Fourier
coefficients. We then need to update c, breaking it down into its Fourier coefficients. We can
then feed both of these into our ergodicity metric in order to see how ergodic our trajectory is.

• We compute the Fourier coefficients. These are based on whatever distribution we have provided
over the domain.


Computation of Fourier Coefficients for φ

begin

var hk := 5, M, N, P := ki;

k is # of Fourier Coefficients, 5 is default

proc phi fourier decompose(xcells, xmin, xsize, ycells, ymin, ysize, zcells, zmin, zsize, domaincellsize, φ[][][])

2≡

for m := 0 to M − 1 step 1 do

for n := 0 to N − 1 step 1 do

for p := 0 to P − 1 step 1 do

coef f [m, n, p] = compute c oef f icients(xcells, xmin, xsize, ycells, ymin, ysize,

zcells, zmin, zsize, domaincellsize, m, n, p, φ)

od

od

od

.


proc phi coefficients(xcells, xmin, xsize, ycells, ymin, ysize, zcells, zmin, zsize,

domaincellsize, m, n, p, φ)

≡
var hi := sqrt(−1), U := 0i;

U is complex valued, i is the complex unit

for xi := 0 to x c ells − 1 step 1 do

x := xmin + (xi − 0.5) ∗ xsize

for yi := 0 to ycells − 1 step 1 do

y := ymin + (yi − 0.5) ∗ ysize

r := sqrt(x 2 + y 2 )

psi := atan2( x y )

bessel := bessel j (m − n, p ∗ r) This is the m-nth Bessel function, with argument p*r

for zi := 0 to zcells − 1 step 1 do

z := zmin + (zi − 0.5) ∗ zsize

U := i n−m ∗ exp(i ∗ (m ∗ psi + (n − m) ∗ z)) ∗ bessel ∗ φ[xi][yi][zi] ∗ domaincellsize

od

od

od

return U

.
Computation of Fourier Coefficients for c:

begin

var hk := 5, M, N, P := k, ck := [][][]i; These should be the same variables as in the phi’s coefficients.

proc c fourier decompose(trajectory[][], N := trajectory.size)

ck is a complex valued matrix.

≡

for m := 0 to M − 1 step 1 do

for n := 0 to N − 1 step 1 do

for p := 0 to P − 1 step 1 do

var hf k s um := 0i;

fk sum is complex valued.

for j := 0 to N − 1 step 1 do

var hxi[] := trajectory[j]i;

f k sum+ = c coef f icients(m, n, p, xi[0], xi[1], xi[2]);

od

ck[m, n, p] = f k N sum

od

od

od

.


3proc c coefficients(m, n, p, x, y, z)

≡

var hr := sqrt(x 2 + y 2 ), ψ := atan2( x y ), i := sqrt(−1)i

i is the complex unit

var hbessel := bessel j (m − n, p ∗ r)i

This is the m-nth Bessel function, with argument p*r

return i n−m ∗ exp(i ∗ (m ∗ ψ + (n − m) ∗ z)) ∗ bessel

.



• We then use the cofficients of both φ and c in our metric in order to see how ergodic our trajectory
is.


Compute Ergodicity:

begin

var hc k := c f ourier decompose(), val := 0, φ k := phi f ourier decompose()i

Λ := U M R()

Λ weights low frequency features heavily

for m := 0 to M − 1 step 1 do

for n := 0 to N − 1 step 1 do

for p := 0 to P − 1 step 1 do

d := φ k [m, n, p] − c k [m, n, p]

dr := real(d)

di := im(d)

val+ = Λ[m, n, p] ∗ (dr 2 + di 2 )

val is our ergodic value.

od

od

od

end


proc UMR(m, n, p, x, y, z, k)

Construction of Unitary Matrix Representation of E 2

begin

M, N, P := k;

k is # of Fourier Coefficients, 5 is default

U [M ][N ][P ] = 0;

for m := 0 to M − 1 step 1 do

for n := 0 to N − 1 step 1 do

for p := 0 to P − 1 step 1 do

1

U [m][n][p] = (1+m 2 +n

2 +p 2 ) 2

od

od

od

return U

.


• Bessel Functions library in c

• NOTE C has native functionality for Bessel functions of order n.
See here: GNU Bessel Functions
