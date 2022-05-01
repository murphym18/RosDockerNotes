# Developing ROS systems with docker

## Using docker and ROS

 My goal is to build the onboard pilot in a docker container.
 With that in mind, I'm trying to get it working on this laptop.

 My laptop doesn't have ROS installed. So as a personal challenge I will do everything in
 containers (just to see if I can).

Here are some basics. To start a ROS container:

```bash
sudo docker run -it --rm ros:noetic-robot 
```

The `ros` image tagged `noetic-robot` is probably the best starting point. It builds on `ros-base`. And `ros-base` builds on `ros-core`.

Here's a summary of these images:

- `ros-core` is the minimal ROS image
- `ros-base` builds on ros-core. It includes basic tools and libraries
- `noetic-robot` matches the [meta package with the same name](https://github.com/ros/metapackages/blob/482da3e297f47a2e06f54d54c16de7e3cb7ec0f4/robot/package.xml). It adds some helpful packages like `smach` and `geometry`

After starting a container, I can use `docker exec` to open another terminal in an
existing container:

```bash
docker exec -it <container name> bash
```

To find the name of a container run:

```bash
docker ps -l
```

Once you're in the container, you can run:

```bash
source ros_entrypoint.sh
```

This sets up the env vars like `$PATH` and enables you to use the usual ROS tools.

When writing this, here's what I did:

1. pulled ros:noetic-robot
2. started a container from the `ros:noetic-robot` image
3. ran `ros core` in the container
4. ran `ros topic list` in the container

## Creating a catkin package using docker software (One-Time setup)

I want to develop an illustrative catkin package. But I don't have ROS or any tools installed.
I think I can use docker to get around that. My plan is to generate the project skeleton
using the ROS dev tools from within a container. Then I can use docker commands to copy
the generated files into this project.

First, I created a ROS container:

```bash
sudo docker run -it --rm ros:noetic-robot 
```

Docker made me a container with ID `598926959f53` (I will use that later).

From within that container, I ran these bash commands:

```bash
source ros_entrypoint.sh
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_create_pkg py_talker rospy std_msgs
```

The last line created a catkin project called `py_talker`. This project depends on `rospy`
and `std_msgs`.

Now that I have the skeleton of a project, I want to copy it to my native file system.
Specifically I want to copy it to the directory holding this README.md file. So back in my
laptop's native bash shell I ran:

```bash
sudo docker cp 598926959f53:/catkin_ws/src/py_talker .
sudo chown `whoami`:`whoami` -R py_talker/
```

And it worked. I copied the `py_talker` directory and all the other files I generated.
`docker cp` reminds me of `scp`.

### Python files

I have a starter catkin project, but I want it to be a python project. So I need to add
python specific files.

Here's what I need to do:

- Create a `setup.py` file. This file will list the scripts and python modules that get
  installed as part of this project.
- Update the `CMakeLists.txt` file. It needs to list all the other files to install. Some files aren't listed in `setup.py` because they're beyond the scope of a python package (`launch` files are a prime example).

Let's try this. I need to create a python module and a script. In my native bash shell:

```bash
mkdir -p py_talker/src/modrn
touch py_talker/src/modrn/__init__.py
touch py_talker/src/modrn/hello.py
```

After that I did the following:

- I put a function in `hello.py` so that I have something to import.
- I created a python script that uses my function. Here's how I made the file in my native
  bash shell:
  ```bash
  mkdir -p bin
  touch bin/hello
  chmod 755 bin/hello
  ```

#### Setup.py

Now that I have a package with some substance (it has a script and a module), I will create a `setup.py` file that
specifies how to install it (somewhat).

**Some Background:**
With ROS, we don't use the standard python packaging tools. But we borrow some of their
conventions. Our `setup.py` file isn't processed by tools such as `distutils`, or
`setuptools`. It's actually processed by `catkin`. For this reason, we must only use
features supported by `catkin`.

Here's the file I need to edit:

```bash
touch py_talker/setup.py
```

Catkin has a special way of handling `setup.py`. You cannot use all the features of
`distutils`. And newer python packaging methods like `setuptools` and `distutils2` are
_not_ supported. Within `setup.py` you can specify:

- version
- scripts (you shouldn't use this feature)
- packages
- package_dir

Here's a boiler plate `setup.py` file with every feature:

```python
from distutils.core import setup

setup(
    version='...',
    scripts=['bin/myscript'],
    packages=['mypkg'],
    package_dir={'': 'src'}
)
```
If you need to do more, then you'll have to use `CMakeLists.txt` and do things the ROS way.


> **Note:** Don't use the `scripts` keyword. It's considered a best practice to not use it.
>Instead, you should specify your scripts in `CMakeLists.txt` so that `rosrun` can be used to invoke it.
> When you use the `scripts` keyword, you can invoke the script even when you have not sourced `setup.bash` for ROS. 

> **Question:** How do you specify python dependencies that get installed with pip? Can `CMakeLists.txt` do that?

For more details about `setup.py` in the context of ROS, check out:

- The [ros wiki](http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules)
- The [catkin docs](https://docs.ros.org/en/hydro/api/catkin/html/user_guide/setup_dot_py.html)

> **Comment:** It seems like catkin uses a `setup.py` file so that python developers can reuse some of their existing knowlege and get up to
speed easier. Beyond that, there is a good amount of logic that goes into the `packages` and `package_dir` keywords. So it makes sensse that we'd want to handle that the pythonic way. 

With `setup.py` out of the way, now we can move on to the stuff that gets specified in `CMakeLists.txt`

#### CMakeLists.txt

The CMakeLists.txt file also needs to be updated for a python package.

I need to uncomment `catkin_python_setup()`:

```
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
```

I need to use the `catkin_install_python` function to install my scripts:

```
catkin_install_python(PROGRAMS bin/hello
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

> **Question:** Can I use `CMakeLists.txt` to specify python dependencies installed using pip?

If I had any launch files, I would also need to install them. For example:

```
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

#### Dockerfile

Next I updated the docker file. I modified it so that the `py_talker` directory gets
copied to `/catkin_ws/src/py_talker`

```docker
FROM ros:noetic-ros-base-focal

COPY py_talker /catkin_ws/src/py_talker
```

I ran docker build:

```bash
sudo docker build -t noetic-py-dev .
```

But during development, I can mount the directory in the container:
```bash
sudo docker run -it --rm -v $PWD/py_talker:/catkin_ws/src/py_talker noetic-py-dev bash
```

The mount is helpful so that my changes are available to the container as I go along.

Then, from within the container, I can build, install and run the scripts:

##### Build

```bash
source ros_entrypoint.sh
cd catkin_ws/
catkin_make
```

##### Install

```bash
source /ros_entrypoint.sh
cd /catkin_ws
catkin_make install
```

##### Run

After you build it, you can run scripts from the package like this
```bash
source /catkin_ws/devel/setup.bash
rosrun py_talker runme.py
```

After you install it, you can run scripts from the package like this:
```bash
source /catkin_ws/install/setup.bash
rosrun py_talker runme.py
```

Congrats! you built and installed a ros package using docker. But, how do I *deploy* a ROS package using docker? 


## Create a Docker Image for my ROS package

I want to create a production-quality docker image for my ROS package. How do I make a minimal
docker image for this project? Beyond that, I think it would be cool if I could
create an image in the cloud with github actions, and have it uploaded to
Backblaze B2, AWS S3 or Cloudflare R2. Either that or I'd like to have it uploaded to a
private docker registry.

I found this article: [Using GitHub Actions to build ARM-based Docker Images](https://medium.com/swlh/using-github-actions-to-build-arm-based-docker-images-413a8d498ee). I note it here for later reading


At this time, here's what I think I'm supposed to do:

1. Make a multi-stage docker file.
1. In the builder stage, create a deb package
1. In the final stage, copy the deb and install it

The problem is I don't know how to use the ROS tooling to create a deb.

Here's another idea. In the builder stage, I run `catkin_make install` then I copy the
install directory to the final image. Here's the Dockerfile:

```docker
FROM ros:noetic-ros-base-focal AS builder

COPY py_talker /catkin_ws/src/py_talker
WORKDIR /catkin_ws
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic

# Final Stage
FROM ros:noetic-ros-base-focal

COPY --from=builder /opt/ros/noetic /opt/ros/noetic

ENTRYPOINT /ros_entrypoint.sh rosrun py_talker runme.py
```

This works! But it has a **big** flaw. It copies the directory `/opt/ros/noetic` and that's a huge
operation. Beyond that, much of the data in there is redundant because it comes with
`ros:noetic-ros-base-focal`.

Maybe, in the builder stage, I could install everything to the `tmp` directory? Lets try that.
Here's the docker file:

```docker
FROM ros:noetic-ros-base-focal AS builder

COPY py_talker /catkin_ws/src/py_talker
WORKDIR /catkin_ws
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/tmp/ros/noetic

# Final Stage
FROM ros:noetic-ros-base-focal

COPY --from=builder /tmp/ros/noetic /opt/ros/noetic

ENTRYPOINT /ros_entrypoint.sh rosrun py_talker runme.py
```

This works too! I changed the install prefix to `/tmp/ros/noetic` and the final stage built
much faster.

This worked quite well. Maybe I don't need to create a deb package. This approach seems
pretty good. When I run `sudo docker history noetic-py-dev`, I can see that my image layer
only takes up 31 kB.

If I were to try and upload this to a registry, is that how much data I would send?
Better yet, if I were to try and pull this image in the field, is that how much data it
would download?
