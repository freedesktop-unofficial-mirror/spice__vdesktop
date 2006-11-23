
include config.mak

DESTDIR=

rpmrelease = devel

.PHONY: kernel user qemu clean

all: $(if $(WANT_MODULE), kernel) user qemu

qemu kernel user:
	$(MAKE) -C $@

qemu: user

clean: 
	@for d in kernel user qemu; do 	\
		$(MAKE) -C $$d $@; 	\
	done

bindir = /usr/bin
bin = $(bindir)/kvm
initdir = /etc/init.d
confdir = /etc/kvm
utilsdir = /etc/kvm/utils

install-rpm:
	mkdir -p $(DESTDIR)/$(bindir)
	mkdir -p $(DESTDIR)/$(confdir)
	mkdir -p $(DESTDIR)/$(initdir)
	mkdir -p $(DESTDIR)/$(utilsdir)
	cp qemu/x86_64-softmmu/qemu-system-x86_64 $(DESTDIR)/$(bin)
	cp scripts/kvm $(DESTDIR)/$(initdir)/kvm
	cp scripts/qemu-ifup $(DESTDIR)/$(confdir)/qemu-ifup
	cp kvm $(DESTDIR)/$(utilsdir)/kvm

install:
	make -C user DESTDIR="$(DESTDIR)" install
	make -C qemu DESTDIR="$(DESTDIR)" install

tmpspec = .tmp.kvm.spec

rpm:	user qemu
	mkdir -p BUILD RPMS/$$(uname -i)
	sed 's/^Release:.*/Release: $(rpmrelease)/' kvm.spec > $(tmpspec)
	rpmbuild --define="kverrel $$(uname -r)" \
		 --define="objdir $$(pwd)" \
		 --define="_topdir $$(pwd)" \
		 --define="prebuilt 1" \
		-bb $(tmpspec)

srpm:
	mkdir -p SOURCES SRPMS
	sed 's/^Release:.*/Release: $(rpmrelease)/' kvm.spec > $(tmpspec)
	tar czf SOURCES/kvm.tar.gz qemu
	tar czf SOURCES/user.tar.gz user
	tar czf SOURCES/kernel.tar.gz kernel
	tar czf SOURCES/scripts.tar.gz scripts
	cp Makefile SOURCES
	rpmbuild  --define="_topdir $$(pwd)" -bs $(tmpspec)

clean:
	for i in $(if $(WANT_MODULE), kernel) user qemu; do \
		make -C $$i clean; \
	done
	rm -f config.make user/config.mak
