# Run in standalone mode

## Content of bin\python

Copy here the content of an embeddable Python package, directly into this folder. It should include a 
`pythonXX._pth` file to list the following search paths (the exact content may vary according to the Python version):

```
python36.zip
.
..\
..\harfang\
..\requests\
..\urllib3\
..\charset_normalizer\
..\certifi\
..\idna\
..\..\app\

# Uncomment to run site.main() automatically
#import site
```