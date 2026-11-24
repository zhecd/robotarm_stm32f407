"""独立核对已导出的完整Flash与指定ELF；本程序只读本地文件。"""
from pathlib import Path
import argparse,hashlib,json
from prepare_acceptance import flash_segments

def verify(elf_path,dump_path):
    elf=Path(elf_path).read_bytes();dump=Path(dump_path).read_bytes()
    if len(dump)!=524288:raise ValueError('full512KiB flash dump required')
    segments=flash_segments(elf);total=0
    for address,offset,length in segments:
        at=address-0x08000000
        if dump[at:at+length]!=elf[offset:offset+length]:raise ValueError('flash load segment mismatch')
        total+=length
    return {'pass':True,'load_bytes':total,'elf_sha256':hashlib.sha256(elf).hexdigest(),
            'full_flash_sha256':hashlib.sha256(dump).hexdigest()}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--elf',required=True);p.add_argument('--dump',required=True);p.add_argument('--output')
    a=p.parse_args();r=verify(a.elf,a.dump);s=json.dumps(r,indent=2)
    if a.output:Path(a.output).write_text(s+'\n',encoding='utf8')
    print(s)
