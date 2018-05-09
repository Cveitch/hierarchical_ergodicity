\noindent
\large
\begin{center}
\fontsize{18}{21.6}
{\bf Ergodic Manual \\}

\fontsize{14}{21.6}
{\bf How to use Ergodicity in Euclidean motion group $E_2$ \\}
\end{center}



\noindent
\normalsize
\begin{center}
{\bf
Conan Veitch

M.Sc Candidate, Computer Science

University of Northern British Columbia

Prince George, BC

veitch@unbc.ca}
\end{center}



\section*{What Are We Doing?}
We are implementing the metric of ergodicity and ergodic dynamics developed by Mathew and Mezi\^{c} in a multi robot system.  In order to do so, we leverage Miller et all's work on projection-based gradient descent in order to optimize the ergodic trajectory of each robot.  Much of this work is based on the Julia implementation of Miller's work provided by Louis Dressel.

The intent of this work is to allow a comparison between the trajectory through a domain of a robot to be compared to an ideal ergodic trajectory, then use this knowledge to make a control decision (which state to enter next).  This is accomplished via a metric that compares the Fourier coefficients of our robots trajectory to the Fourier coefficients of the ideal trajectory over our domain.  This result is then optimized via gradient descent.
\subsection*{How Do We Accomplish This?}
There are a series of steps we must take in order to have a robot follow an ergodic search trajectory:
\begin{itemize}
	\item Find a domain $D$.  We must have a rectangular domain in $\mathbb{R}^n$.  We will use the group $E_2$ (described below) as our domain, as it fulfils all requirements.
	\item Find a distribution $\phi$ over $D$.  We must have a distribution over our domain that we call the spatial distribution.  This distribution represents information density over $D$.
	\item A fourier basis function $F_k(x)$ for $D$.  Since $E_2$ is our domain, we can use the basis function that Miller and Murphy have provided.  This basis function is in terms of the Bessel function, which is itself sinusoidal - allowing for it to act as a Fourier basis function.
	\item We must then decompose $\phi$ into Fourier coefficients - possible since we have described $\phi$ in terms of the Bessel function.
	\item The trajectory of the robot must then be described as distribution $c$ over the domain, then decomposed into Fourier coefficients.  $c$ is described in terms of the Dirac delta function.
	\item We then call $\varepsilon$ the sum of squared differences of $c$ and $\phi$, weighted by a factor $\Lambda$.  $\varepsilon$ is our metric of ergodicity.
\end{itemize}

\section*{What is $E_2$?}
$E_2$ is the special Euclidean motion group in two dimensions.  Each element in this group can be thought of as a state in time: an x coordinate, a y coordinate, and an angle $\theta$ (this angle is measured from $\pi$, counter clockwise).  It is the group of isometries of Euclidean space in two dimensions ($R^2$).  That is, it's elements are linear transformations that preserve length - rotation and translation.  The operation on this group is function composition (eg: translation, and rotation).  

\subsection*{Why Bother With $E_2$?}
The reason we have chosen this particular domain for our ergodic search domain is because we are implementing a metric of ergodicity physically - in a multi robot system.  Given that our robots have positions and bearings, $E_2$ then supplies us with a domain that caters to our multi robot systems' construction.  An infrared detector is positioned in the front of each robot, and as such the detector expects information to be supplied in that position.

\section{Steps for Ergodicity in $E_2$}
\begin{itemize}
	\item Create domain $D$.  The $D$ is a structure that contains all of the states of $E_2$ that we are interested in.  In this way we restrict $D$'s boundaries to some rectangle in $R^2$.
	
	\begin{itemize}
		\item Create a domain data structure that contains information about $D$.
		\item Domain structure contains the bottom left and top right boundary coordinates the rectangular space that makes up $D$.
		\item We are using $E_2$ as $D$, so we also need to add the angles $-\pi$ to $\pi$ in order to cover the entire angular space.
		\item $D$ can then be described as a vector $[r, \psi, \theta]^T$ using polar coordinates.
		\item Discretize $D$.  Split the domain into discrete sub-rectangles.  Given a domain that is p meters X q meters, we divide that into $n$ smaller, but identical squares.  Eg: You could split a 1m x 1m square into 100 1cm x 1cm chunks.
	\end{itemize}	
	 
	\item Create a probability distribution $\phi$ over this domain - $\phi$ denotes information density over the domain.  Coordinates that we would like our trajectory to spend more time within will be weighted heavier than coordinates we do not want to spend time within.
	\begin{itemize}
		\item Represent $\phi$ as an array with $n$ dimensions ($n=2$ for $E_2$).  The length of each dimension of the array is equal to the number of sub-rectangles we discretized the domain into.
		\item $\phi$ is created via functions with the domain as an argument.  We require a function in order to populate the array and one to normalize the data. Initially we require only that a single circular area of high information density exists, but we will need to implement a way to create gaussians eventually in order to create arbitrary PDFs (arbitrary, but still normal).
	\end{itemize}
	
\end{itemize}

\begin{itemize}
	\item Ergodic Construction.  An ergodic controller must be created that takes $D$, $\phi$, and the number of Fourier coefficients of $\phi$. We will need a way to update $\phi$, breaking it down into its Fourier coefficients.  We then need to update $c$, breaking it down into its Fourier coefficients.  We can then feed both of these into our  ergodicity metric in order to see how ergodic our trajectory is. 
	


	
\end{itemize}

\end{multicols}

\hrulefill
	\begin{itemize}
		\item We compute the Fourier coefficients.  These are based on whatever distribution we have provided over the domain.
	\end{itemize}
	
	\begin{program}
		\mbox{Computation of Fourier Coefficients for $\phi$}
		\BEGIN \\ %
		\VAR \seq{k:=5, M, N, P:= k}; %
		\rcomment{k is \# of Fourier Coefficients, 5 is default}
		\PROC |phi_fourier_decompose|(xcells, xmin, xsize, ycells, ymin, ysize, zcells, zmin, zsize, domaincellsize, \phi [][][]) 
		\BODY
		\FOR m:=0 \TO M -1 \STEP 1 \DO
			\FOR n:=0 \TO N -1 \STEP 1 \DO
				\FOR p:=0 \TO P -1 \STEP 1 \DO
					coeff[m, n, p] = compute_coefficients(xcells, xmin, xsize, ycells, ymin, ysize, 
					 \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad zcells, zmin, zsize, domaincellsize, m, n, p, \phi)
				\OD
			\OD
		\OD
		\ENDPROC \\

		\PROC |phi_coefficients|(xcells, xmin, xsize, ycells, ymin, ysize, zcells, zmin, zsize, 
		\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad domaincellsize, m, n, p, \phi) 
		\BODY
		\VAR \seq{i:=sqrt(-1), U:=0};%
		\rcomment{U is complex valued, i is the complex unit}		
		\FOR xi:=0 \TO x_cells -1 \STEP 1 \DO
			x:=xmin+(xi-0.5)*xsize
			\FOR yi:=0 \TO ycells -1 \STEP 1 \DO
				y:=ymin+(yi-0.5)*ysize
				r:=sqrt(x^2+y^2)
				psi:=atan2(\frac{y}{x})
				bessel:=bessel_j(m-n, p*r)%
				\rcomment{This is the m-nth Bessel function, with argument p*r}
				\FOR zi:=0 \TO zcells -1 \STEP 1 \DO
					z:=zmin+(zi-0.5)*zsize
					U:= i^{n-m} * exp(i*(m*psi+(n-m)*z)) * bessel * \phi [xi][yi][zi] * domaincellsize
				\OD
			\OD
		\OD
		return \: U
		\ENDPROC
					
	\end{program}
		
		
		
\begin{program}
		\mbox{Computation of Fourier Coefficients for $c$:}
		\BEGIN \\ %
		\VAR \seq{k:=5, M, N, P:= k, ck:=[][][]}; %
		\rcomment{These should be the same variables as in the phi's coefficients.}
		\PROC |c_fourier_decompose|(trajectory[][], N:= trajectory.size)%
		\rcomment{ck is a complex valued matrix.}
		\BODY		
		\FOR m:=0 \TO M -1 \STEP 1 \DO
			\FOR n:=0 \TO N -1  \STEP 1 \DO
				\FOR p:=0 \TO P -1 \STEP 1 \DO
					\var \seq{fk_sum:=0};%
					\rcomment{fk\_sum is complex valued.}
					\FOR j:=0 \TO N-1 \STEP 1 \DO
						\var \seq{xi[]:=trajectory[j]};
						fk\_sum += c\_coefficients(m, n, p, xi[0], xi[1], xi[2]);
					\OD
					ck[m, n, p] = \frac{fk\_sum}{N}
				\OD
			\OD
		\OD
		\ENDPROC
		
		\PROC |c_coefficients|(m, n, p, x, y, z)
		\BODY
			\VAR \seq{r:=sqrt(x^2 + y^2), \psi :=atan2(\frac{y}{x}), i:=sqrt(-1)}%
			\rcomment{i is the complex unit}\\
			\VAR \seq{bessel:=bessel_j(m-n, p*r)}%
			\rcomment{This is the m-nth Bessel function, with argument p*r}\\
			return \: i^{n-m} * exp(i*(m*\psi + (n-m)*z)) * bessel
		\ENDPROC
	\end{program}
		
		
		
\begin{itemize}
		\item We then use the cofficients of both $\phi$ and $c$ in our metric in order to see how ergodic our trajectory is.
	\end{itemize}
		
		
	\begin{program}
		\mbox{Compute Ergodicity:}
		\BEGIN \\ %
		\VAR \seq{c_k:=c\_fourier\_decompose(), val:=0, \phi_k:=phi\_fourier\_decompose()}
		\Lambda:= UMR()%
		\rcomment{$\Lambda$ weights low frequency features heavily}
		\FOR m:=0 \TO M -1 \STEP 1 \DO
			\FOR n:=0 \TO N -1 \STEP 1 \DO
				\FOR p:=0 \TO P -1 \STEP 1 \DO
					d:=\phi_k[m, n, p] - c_k[m, n, p]
					dr:= real(d)
					di:= im(d)
					val += \Lambda[m, n, p] * (dr^2 + di^2)% 
					\rcomment{val is our ergodic value.}
				\OD
			\OD
		\OD
		\END
		
		
		
		
		\PROC |UMR|(m, n, p, x, y, z, k)%
		\rcomment{Construction of Unitary Matrix Representation of $E_2$}
		\BEGIN \\ %
		M, N, P:= k; %
		\rcomment{k is \# of Fourier Coefficients, 5 is default}
		U[M][N][P] = { 0 };
		\FOR m:=0 \TO M -1 \STEP 1 \DO
			\FOR n:=0 \TO N -1 \STEP 1 \DO
				\FOR p:=0 \TO P -1 \STEP 1 \DO
					U[m][n][p] = \frac{1}{(1+m^2 + n^2 + p^2)^2}
				\OD
			\OD
		\OD
		return \: U
	\ENDPROC
	\end{program}	
	
	
\begin{itemize}
	\item \href{https://www.atnf.csiro.au/computing/software/gipsy/sub/bessel.c}{Bessel Functions library in c}
	
	\item NOTE the above may be unnecessary, c has native functionality for Bessel functions of order n. See here: \href{http://www.gnu.org/software/libc/manual/html_node/Special-Functions.html}{GNU Bessel Functions}
\end{itemize}
