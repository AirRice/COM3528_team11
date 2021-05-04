# COM3528_team11

To get basic functions code working, ensure you have cloned the basic functions repo inside your mdk core folder. Then import like so: 
```
cd ~/mdk/share/python/miro2/core

python

>>> from basic_functions import miro_ros_interface as mri

```

## Code Template
To copy code to the miro

```scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no myfile.py miro@192.168.x.x:/home/miro```

Replace ```myfile.py``` with the file you want to use

Replace ```x.x``` in ```miro@192.168.x.x:/home/miro``` with the relevant parts of your miro IP

To run the code

```ssh miro@192.168.x.x```

Default ssh password - miro

```python myfile.py```
