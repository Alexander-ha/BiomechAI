import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class FileNameService {
  public fileName: string = '';

  setFileName(name: string): void {
    this.fileName = name;
  }
}
