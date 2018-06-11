from collections import OrderedDict
import os

from .friendly_path_imp import friendly_path
from .locate_files_impl import locate_files
from .logging_logger import logger
from .memoization import memoize_simple
from .mkdirs import d8n_make_sure_dir_exists
from .path_utils import get_ros_package_path
from .paths import get_duckietown_cache_dir
from .system_cmd_imp import system_cmd_result
from .test_hash import get_md5, parse_hash_url
from .text_utils import indent
from .yaml_pretty import yaml_load_plain


#from .paths import get_duckiefleet_root, get_duckietown_root, \
#    get_duckietown_data_dirs
@memoize_simple
def get_dropbox_urls():
    logger.info('Getting urls...')
    sources = []
#    sources.append(get_duckiefleet_root())
#    sources.append(get_duckietown_root())
#    sources.extend(get_duckietown_data_dirs())
    sources.append(get_ros_package_path('easy_logs'))

    found = []
    urls = OrderedDict()
    for s in sources:
        pattern = '*.urls.yaml'

        filenames = locate_files(s, pattern, case_sensitive=False)
        for f in filenames:
            found.append(f)
            logger.debug('loading %s' % f)
            data = open(f).read()
            f_urls = yaml_load_plain(data)
            for k, v in f_urls.items():
                urls[k] = v

    msg = 'Found %d urls in %s files:\n' % (len(urls), len(found))
    msg += '\n'.join(found)
    logger.info(msg)

    def sanitize(url):
        if url.endswith('?dl=0'):
            url = url.replace('?dl=0', '?dl=1')
        return url

    return dict([(k, sanitize(url)) for k, url in urls.items()])


def download_if_not_exist(url, filename):
    if not os.path.exists(filename):
        logger.info('Path does not exist: %s' % filename)
        download_url_to_file(url, filename)
        if not os.path.exists(filename):
            msg = 'I expected download_url_to_file() to raise an error if failed.'
            msg += '\n url: %s' % url
            msg += '\n filename: %s' % filename
            raise AssertionError(msg)
    return filename


def download_url_to_file(url, filename):
    logger.info('Download from %s' % (url))
    tmp = filename + '.tmp_download_file'
    cmd = [
        'wget',
        '-O',
        tmp,
        url
    ]
    d8n_make_sure_dir_exists(tmp)
    res = system_cmd_result(cwd='.',
                          cmd=cmd,
                          display_stdout=False,
                          display_stderr=False,
                          raise_on_error=True,
                          write_stdin='',
                          capture_keyboard_interrupt=False,
                          env=None)

    if not os.path.exists(tmp) and not os.path.exists(filename):
        msg = 'Downloaded file does not exist but wget did not give any error.'
        msg += '\n url: %s' % url
        msg += '\n downloaded to: %s' % tmp
        msg += '\n' + indent(str(res), ' | ')
        d = os.path.dirname(tmp)
        r = system_cmd_result(d, ['ls', '-l'], display_stdout=False,
                          display_stderr=False,
                          raise_on_error=True)
        msg += '\n Contents of the directory:'
        msg += '\n' + indent(str(r.stdout), ' | ')
        raise Exception(msg)

    if not os.path.exists(filename):
        os.rename(tmp, filename)

    logger.info('-> %s' % friendly_path(filename))


def get_file_from_url(url):
    """
        Returns a local filename corresponding to the contents of the URL.
        The data is cached in caches/downloads/
    """
    basename = get_md5(url)
    if 'jpg' in url:
        basename += '.jpg'

    cachedir = get_duckietown_cache_dir()
    filename = os.path.join(cachedir, basename)
    download_if_not_exist(url, filename)
    return filename


@memoize_simple
def get_sha12url():
    sha12url = {}
    urls = get_dropbox_urls()
    for u, v in urls.items():
        u = unicode(u)  #.encode('utf-8')
#        print('%s' % u)
        if u.startswith('hash:'):
            parsed = parse_hash_url(u)
            sha12url[parsed.sha1] = v
    return sha12url

#
#def require_resource_from_hash_url(hash_url, destination=None):
#    url = resolve_url(hash_url)
#
#    if destination is None:
#        parsed = parse_hash_url(hash_url)
#        basename = parsed.name if parsed.name is not None else parsed.sha1
#        dirname = get_duckietown_cache_dir()
#        destination = os.path.join(dirname, basename)
#
#    d8n_make_sure_dir_exists(destination)
#    download_if_not_exist(url, destination)
#    return destination
#
#
#def resolve_url(hash_url):
#    urls = get_dropbox_urls()
#    parsed = parse_hash_url(hash_url)
#    sha12url = get_sha12url()
#    if parsed.sha1 in sha12url:
#        logger.info('found match via sha1: %s' % parsed.sha1)
#        return sha12url[parsed.sha1]
#    else:
#        if parsed.name in urls:
#            logger.info('found match via name: %s' % parsed.name)
#            return urls[parsed.name]
#        else:
#            msg = 'Cannot find url for %r' % hash_url
#            raise Exception(msg)


def require_resource(basename, destination=None):
    """ Basename: a file name how it is in urls.yaml

        It returns the URL.
    """
    urls = get_dropbox_urls()
    if not basename in urls:
        msg = 'No URL found for %r.' % basename
        raise Exception(msg)
    else:
        url = urls[basename]
        if destination is None:
            dirname = get_duckietown_cache_dir()
            destination = os.path.join(dirname, basename)
        d8n_make_sure_dir_exists(destination)
        download_if_not_exist(url, destination)
        return destination
