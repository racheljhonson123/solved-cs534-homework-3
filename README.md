Download Link: https://assignmentchef.com/product/solved-cs534-homework-3
<br>
<h1>Part I: Camera Calibration using 3D calibration object</h1>

We want to calibrate the camera of a robot vehicle. We will use a linear method as described in the lectures. We place a large cubic frame of size 4 meters on the road several meters in front of the vehicle. The positions of the eight corners of the cubic frame are de ned with respect to a world coordinate system with its axes parallel to the cube edges and with its origin at the center of the cube. The world coordinates of the cube vertices are:

2 2 2,

-2 2 2,

-2 2 -2,

2 2 -2,

2 -2 2,

-2 -2 2,

-2 -2 -2,

2 -2 -2

We detect the corresponding cube corners at the following pixel positions in the camera image:

422 323; 178 323; 118 483; 482 483; 438 73; 162 73; 78 117; 522 117

<ol>

 <li>Draw the image points, using small circles for each image point.</li>

 <li>Write a Matlab function that takes as argument the homogeneous coordinates of one cube corner andthe homogeneous coordinates of its image, and returns 2 rows of the matrix P (slide 30 of the Camera Calibration pdf document). This matrix P will be used to compute the 12 elements of the projection matrix <strong>M </strong>such that <em>λp<sub>i </sub></em>= <strong>M</strong><em>P<sub>i</sub></em></li>

 <li>Use this Matlab function to generate 2 rows of the matrix P for each cube corner and its image and obtain a matrix with 16 rows and 12 columns. Print matrix P.</li>

 <li>Now we need to solve the system P<em>m </em>= 0. Find the singular value decomposition of matrix P using matlab svd function. The last column vector of V obtained by svd(P) should be the 12 elements in row order of the projection matrix that transformed the cube corner coordinates into their images. Print the matrix <strong>M</strong>.</li>

 <li>Now we need to recover the translation vector which is a null vector of <strong>M</strong>. Find the singular value decomposition of matrix <strong>M </strong>= <em>U</em>Σ<em>V <sup>T</sup></em>. The 4 elements of the last column of <em>V </em>are the homogeneous coordinates of the position of the camera center of projection in the frame of reference of the cube (as in slide 36). Print the corresponding 3 Euclidean coordinates of the camera center in the frame of reference of the cube.</li>

 <li>Consider the 3×3 matrix <strong>M</strong>’ composed of the rst 3 columns of matrix <strong>M</strong>. Rescale the elements of this matrix so that its element <em>m</em><sub>33 </sub>becomes equal to 1. Print matrix <strong>M</strong>’ . Now let the rotation matrices be as de ned in slide 38 where the axes e1, e2, e3 are the <em>x,y,z </em>axes respectively. Therefore <strong>M</strong>’ can be written as <strong>M</strong></li>

 <li>We will perform the RQ factorization of <strong>M</strong>’ in several steps. First, nd a rotation matrix <em>R<sub>x </sub></em>that sets the term at position (3,2) to zero when <em>R<sub>x </sub></em>is multiplied to <strong>M</strong>’. The cosine and sine used in this matrix are of the form</li>

</ol>

<em>cos</em>(<em>θ<sub>x</sub></em>) = <em>m</em><sub>33</sub><em>/sqrt</em>(<em>m</em><sup>2</sup><sub>33 </sub>+ <em>m</em><sup>2</sup><sub>32</sub>)

<em>sin</em>(<em>θ<sub>x</sub></em>) = −<em>m</em><sub>32</sub><em>/sqrt</em>(<em>m</em><sup>2</sup><sub>33 </sub>+ <em>m</em><sup>2</sup><sub>32</sub>)

Note that the term at position (3, 2) would also be set to zero if the signs of <em>cos</em>(<em>θ<sub>x</sub></em>) and <em>sin</em>(<em>θ<sub>x</sub></em>) were reversed, but this would lead to nding a negative focal length for the camera. So we should choose the signs that leads to a positive focal length. Compute the angle <em>θ<sub>x </sub></em>of this rotation in degrees. Compute matrix <strong>N </strong>= <strong>M</strong>’ ∗ <em>R<sub>x</sub></em>. Print <em>R<sub>x</sub></em>, <em>θ<sub>x </sub></em>and <strong>N</strong>.

<ol start="8">

 <li>The element <em>n</em><sub>31 </sub>of <strong>N </strong>is small enough so that there is no need for a rotation <em>R<sub>y</sub></em>. However, element <em>n</em><sub>21 </sub>is large and a rotation matrix <em>R<sub>z </sub></em>is needed to set it to zero. Compute the rotation matrix <em>R<sub>z </sub></em>using cosine and sine of the form</li>

</ol>

<em>cos</em>(<em>θ<sub>z</sub></em>) = <em>n</em><sub>22</sub><em>/sqrt</em>(<em>n</em><sup>2</sup><sub>21 </sub>+ <em>n</em><sup>2</sup><sub>22</sub>)

<em>sin</em>(<em>θ<sub>z</sub></em>) = −<em>n</em><sub>21</sub><em>/sqrt</em>(<em>n</em><sup>2</sup><sub>21 </sub>+ <em>n</em><sup>2</sup><sub>22</sub>)

Compute the rotation angle <em>θ<sub>z </sub></em>in degrees. This angle is actually very small.

<ol start="9">

 <li>Since we factorized out <em>R<sub>z </sub></em>we can directly compute the calibration matrix <em>K</em>, how? Compute <em>K </em>and rescale so that its element <em>K</em><sub>33 </sub>is set to 1. Print <em>K</em>. What are the focal lengths of the camera in pixels? What are the pixel coordinates of the image center of the camera?</li>

</ol>

<h1>Part II: Camera Calibration using 2D calibration object</h1>

In this part we are going to implement camera calibration from multiple images of 2D planes. Additionally we will learn how to augment images with virtual objects. We will follow the method described in the book chapter on camera calibration by Zhengyou Zhang which was proposed in his paper Flexible Camera Calibration by Viewing a Plane from Unknown Orientations – Zhang, ICCV99 . Start by carefully reading Section 2.4 of that Chapter.

<h2>Calibration Grid and images</h2>

We know the following about the grid. The grid is 9 squares in width and 7 in height. Each square is 30mm x 30mm. If we select the bottom left corner of the grid to be the origin of the world coordinate system, and the grid to be the plane corresponding to Z=0 , then we know the 3D coordinates of each corner in that grid.

For calibration we will use four images: images2.png, images9.png, images12.png, images20.png

<h2>Corner Extraction and Homography computation (10 points)</h2>

First we want to extract the four corners of the calibration grid form each image. We will use the grid corners to estimate the homographies relating two images. We can manually get the four grid corners from each image. One way to let a user manually select points in matlab is using ginput function.

Once the 4 corners are extracted, compute the homography <em>H </em>that relates the grid 3d coordinates to the corners. Use the function homography2d which is provided. Repeat this for the four images provided. Report the computed <em>H </em>for each image [deliverable].

<h2>Computing the Intrinsic and Extrinsic parameters (30 points)</h2>

Now given the four homographies, follow the instructions in section 2.4.4 to compute the intrinsic parameters and extrinsic parameters. We need to linearize the two constraints in Eq 2.19 and 2.20 into two equations in a homogeneous system as in Eq 2.25. Then solve for <em>b </em>and estimate the intrinsic parameters as described in page 21. Print the computed matrix <strong>B </strong>and the intrinsic parameters. Then compute and print <strong>R</strong><em>,t </em>for each image [deliverable].

Verify that your rotation matrix is in fact a rotation matrix, print <em>R<sup>T</sup>R</em>, is it an identity as it should be? We can enforce <strong>R </strong>to be a rotation matrix by SVD decomposition of <strong>R </strong>and setting the singular values to ones, i.e., set the rotation matrix to <em>UV <sup>T </sup></em>where <em>R </em>= <em>U</em>Σ<em>V <sup>T</sup></em>.

Print the new <em>R </em>and <em>R<sup>T</sup>R </em>after enforcing the rotation matrix constraints. [deliverable]

<h2>Improving accuracy (30 points)</h2>

Since we used four manually entered points to compute the homography. A small error in one of the points will directly e ect the computed homography. To      x this we are going to estimate the homography from all grid points.

<ul>

 <li>First given the computed homographies from Section 2, compute the approximate location of each grid corner in the image. (Hint : This can be done since we know the 3d locations of the grid corners and the approximate homography. Call these points p_approx. Create a gure with the image and approximate grid locations. Call this Figure 1 : Projected grid corners [deliverable]</li>

 <li>Second, using the provided Harris function detect Harris corners in the image and display them. Use the following parameter values for the Harris detection : sigma = 2, thresh = 500, radius = 2.</li>

</ul>

[cim<em>,</em>r<em>,</em>c<em>,</em>rsubp<em>,</em>csubp] = harris(rgb2gray(im)<em>,</em>sigma<em>,</em>thresh<em>,</em>radius<em>,</em>disp);

Here r is the y-coordinate of the Harris corner, c is the x-coordinate of the Harris corner, rsubp is the y coordinate with subpixel accuracy, csubp is the x coordinate with subpixel accuracy. Use rsubp, csubp. Create a gure with image and overlayed Harris corners. Call this Figure 2 : Harris corners . [deliverable]

<ul>

 <li>Third, compute the closest Harris corner to each approximate grid corner. (You may nd it useful to use the dist2.m function provided). Let these closest Harris corners be p_correct. Create a gure with the image and p_correct overlayed. Call this Figure 3 : grid points . [deliverable]</li>

 <li>Finally, compute a new homography from p_correct, print H [deliverable]</li>

 <li>Repeat this for the other three images. Then use the homographies to estimate <strong>K </strong>and <strong>R</strong>, <em>t </em>for each image. Report your <strong>K</strong>, <strong>R</strong>’s, and <em>t</em>’s [deliverable]. Save your results, you will need to use them in Part III</li>

 <li>Using the new computed H, compute the errors between points in p_correct and points you get by projecting grid corners to the image (Hint there is no need to use R, t for projecting) . Call this err_reprojection. Report your result. [deliverable]</li>

 <li>Now repeat the process using 4 images. Compare your results to your previous results and those of part 2 [deliverable].</li>

 <li>Can you suggest a way this can be done automatically (i.e without rst letting the user manually select the 4 corners) ?</li>

</ul>

<h1>Part III: Augmented Reality 101</h1>

<h2>Augmenting an Image (30 points)</h2>

Now we would like to use our computed homographies from part II to map a clip art image onto the grid such that it seems to be part of the grid. The image should be synthesized such that the clip art bottom left corners is the same as the grids (0,0) corner. When tting the clip art you should rescale it to t the grid while keeping the clip art aspect ratio. Using your computed homography nd a way to map your image on the grid such that you image will have the same projective distortion as the grid. If the clip art have any white pixels you should make these pixels appears transparent when overlayed over the grid. For each image of the four images in Part II, create a gure with the original image and your virtual clip art overlayed over the grid. Your image should be one of the images provided in the clipart directory. To nd which clip art you are supposed to map, take the last 4 digits of your ruid id add them up and use the clip art le corresponding to the rst digit in the sum, i.e., if the rst 4 digits in your RUID are 5243, use the clip art 4.xxx .

<h2>Augmenting an Object (20 points)</h2>

Now we would like to augment our images with 3D objects. For our purposes we are going to use a cube as a virtual object. We will only render the cube as a wire frame and we would like its base to be locate on the 3×3 grid of squares in the bottom left corner of our grid. The cube should be standing up from the grid. First print the 3D coordinates of the cube. Then, nd a way to use your computed <strong>H</strong>, <strong>K</strong>, <strong>R</strong>, t to synthesize new images with the virtual cube inserted.

<h2>Extra credit (20 points)</h2>

Do one of the following :

<ol>

 <li>Instead of augmenting a cube, augment a general mesh from a 3D le of your choice.</li>

 <li>Can you nd a way to estimate the intrinsic and extrinsic parameters from only two images of the grid. What assumptions on the intrinsic parameters are needed to achieve this. (Hint the answer can be found in Sec 2.4)</li>

</ol>