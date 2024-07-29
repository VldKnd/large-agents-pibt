This is repository with implementations of [priority inheritance backtracking](https://kei18.github.io/pibt2/) for large agents on square grid (LaPIBT).

**Here is an example of algorithms solutions**

<details>
<summary>Hello! Solved by LaPIBT</summary>
<br/>
<div class="image-container">
    <img style="display: none;" id="spinner" src="solution-visualisation/videos/hello.gif"/>
</div>  
</details>


## Project Structure:
Project has two realisations of code, that follow structure, very similar to original [PIBT](https://github.com/Kei18/pibt2) code. We also use same third-party utilities.

**/circle-large-agents-mapf**

> You will find realisation of LaPIBT for object, that have circular shapes. Folder also contains example of solution in .gif format, examples of test cases and instractions on how to build and run code.

**/square-large-agents-mapf**

> You will find realisation of LaPIBT for object, that have square shapes. Folder also contains example of solution in .gif format, examples of test cases and instractions on how to build and run code.

**/third_party**

> Folder contains necessary third party libraries in form of _git-submodules_ for both square and circlular realisations. It should not really be looked at, but its important to pull/clone repository with its submodules to populate this folder
>
> e.g. git pull --recurse-submodules or  git clone --recurse-submodules

**/solution-visualisation**

> This folder has utility code to create visualize and create gifs out of solution files from both circular and square realisation of LaPIBT.
