/* Standalone loader for the touch -> HID bridge.
 *
 * The macan line boots real launchd, so the bridge cannot ride in as an
 * injection into a platform binary the way it did on the HackiOS root.  Its
 * translation unit starts the forwarding thread from a library constructor, so
 * the daemon's whole job is to dlopen it and stay alive: without a live client
 * the virtual digitizer disappears from the event server again.
 */
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

int
main(void)
{
	const char *paths[] = { "/redwood/lib/libredwood.dylib", "/redwood/LaunchDaemons/libredwood.dylib" };
	unsigned i;
	for (i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
		void *h = dlopen(paths[i], RTLD_NOW);
		printf("redwood-inputd: %s -> %p\n", paths[i], h);
		if (h != NULL) {
			for (;;) {
				sleep(3600);
			}
		}
	}
	printf("redwood-inputd: no bridge loaded\n");
	return 1;
}
